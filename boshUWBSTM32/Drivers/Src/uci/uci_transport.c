/*
 * uci_transport.c
 *
 * SPI transport layer for UCI communication with SR250.
 *
 * Wraps existing nucleo_spi_write/read_both functions with:
 *   - Direction byte protocol (0x00=write, 0xFF=read)
 *   - Two-phase read (header first, then payload)
 *   - NXP extended payload length support (>255 bytes)
 *   - Inter-transaction timing delays from reference code
 *   - Hardware reset via RST_N pin
 */

#include "uci/uci_transport.h"
#include "uci/uci_defs.h"
#include "spi.h"
#include "gpio.h"
#include "main.h"
#include <string.h>

/* ========================================================================== */
/*  DWT Cycle Counter for Microsecond Delays                                  */
/* ========================================================================== */

static void uci_dwt_init(void)
{
    /* Enable the DWT trace unit */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset and enable the cycle counter */
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void uci_delay_us(uint32_t us)
{
    /* SystemCoreClock = 480MHz → 480 cycles per µs */
    uint32_t cycles = us * (SystemCoreClock / 1000000UL);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles)
        ;
}

/* ========================================================================== */
/*  Static Buffers                                                            */
/* ========================================================================== */

/* TX buffer for send: 1 direction byte + max UCI packet */
static uint8_t s_tx_buf[1 + UCI_HDR_SIZE + UCI_MAX_EXT_PAYLOAD_SIZE];

/* Buffers for receive phase 2: 1 direction byte + max payload */
static uint8_t s_rx_p2_buf[1 + UCI_MAX_EXT_PAYLOAD_SIZE];
static uint8_t s_tx_p2_buf[1 + UCI_MAX_EXT_PAYLOAD_SIZE];

/* ========================================================================== */
/*  Public API                                                                */
/* ========================================================================== */

void uci_transport_init(void)
{
    uci_dwt_init();
}

/* -------------------------------------------------------------------------- */
/*  Send                                                                      */
/* -------------------------------------------------------------------------- */

uci_status_t uci_transport_send(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) {
        return UCI_STATUS_INVALID_PARAM;
    }
    if (len > (UCI_HDR_SIZE + UCI_MAX_EXT_PAYLOAD_SIZE)) {
        return UCI_STATUS_INVALID_MSG_SIZE;
    }

    /* Prepend direction byte 0x00 (write) */
    s_tx_buf[0] = UCI_DIR_BYTE_WRITE;
    memcpy(&s_tx_buf[1], data, len);

    uint32_t ret = nucleo_spi_write(s_tx_buf, (uint16_t)(1 + len), UCI_SPI_TIMEOUT_MS);

    /* Post-write delay: 80µs per reference code timing */
    uci_delay_us(UCI_POST_WRITE_DELAY_US);

    return (ret == 0) ? UCI_STATUS_OK : UCI_STATUS_TRANSPORT_ERROR;
}

/* -------------------------------------------------------------------------- */
/*  Receive (Two-Phase Read)                                                  */
/* -------------------------------------------------------------------------- */

uci_status_t uci_transport_receive(uint8_t *buffer, uint16_t *out_len, uint16_t max_len)
{
    if (buffer == NULL || out_len == NULL || max_len < UCI_HDR_SIZE) {
        return UCI_STATUS_INVALID_PARAM;
    }

    /* Pre-read delay: 500µs per reference code timing */
    uci_delay_us(UCI_PRE_READ_DELAY_US);

    /* ── Phase 1: Read 4-byte UCI header ── */
    /* Send: [0xFF(dir), 0x00, 0x00, 0x00, 0x00] → Receive: [echo, oct0, oct1, oct2, oct3] */
    uint8_t tx_hdr[5] = { UCI_DIR_BYTE_READ, 0x00, 0x00, 0x00, 0x00 };
    uint8_t rx_hdr[5];

    uint32_t ret = nucleo_spi_read_both(tx_hdr, rx_hdr, 5, UCI_SPI_TIMEOUT_MS);
    if (ret != 0) {
        return UCI_STATUS_TRANSPORT_ERROR;
    }

    /*
     * Map received bytes to UCI header octets:
     *   rx_hdr[0] = direction byte echo (discard)
     *   rx_hdr[1] = Octet 0: MT[7:5] | PBF[4] | GID[3:0]
     *   rx_hdr[2] = Octet 1: EXT[7] | RFU[6] | OID[5:0]
     *   rx_hdr[3] = Octet 2: RFU (standard) or Payload Len LSB (extended)
     *   rx_hdr[4] = Octet 3: Payload Length MSB
     */
    buffer[0] = rx_hdr[1];
    buffer[1] = rx_hdr[2];
    buffer[2] = rx_hdr[3];
    buffer[3] = rx_hdr[4];

    /* Parse payload length, checking NXP extended length bit */
    uint16_t payload_len;
    if (rx_hdr[2] & UCI_EXT_LEN_BIT) {
//    	if (rx_hdr[2] == 0x8A) {
//    		payload_len = ((uint16_t)rx_hdr[4] << 8) | (uint16_t)rx_hdr[3];
//    	}
        /* Extended: 16-bit payload length from octets 2-3 */
        payload_len = ((uint16_t)rx_hdr[4] << 8) | (uint16_t)rx_hdr[3];
    } else {
        /* Standard: 8-bit payload length from octet 3 only */
        payload_len = (uint16_t)rx_hdr[4];
    }

    /* No payload — header-only packet (e.g. some status responses) */
    if (payload_len == 0) {
        *out_len = UCI_HDR_SIZE;
        return UCI_STATUS_OK;
    }

    /* Validate output buffer can hold full packet */
    if ((UCI_HDR_SIZE + payload_len) > max_len) {
        *out_len = UCI_HDR_SIZE;
        return UCI_STATUS_INVALID_MSG_SIZE;
    }

    /* Validate payload does not exceed our static buffer */
    if (payload_len > UCI_MAX_EXT_PAYLOAD_SIZE) {
        *out_len = UCI_HDR_SIZE;
        return UCI_STATUS_INVALID_MSG_SIZE;
    }

    /* Inter-phase delay: 50µs per reference code timing */
    uci_delay_us(UCI_POST_READ_DELAY_US);

    /* ── Phase 2: Read payload ── */
    /* Send: [0xFF(dir), 0x00 * payload_len] → Receive: [echo, payload...] */
    uint16_t phase2_len = 1 + payload_len;

    memset(s_tx_p2_buf, 0x00, phase2_len);
    s_tx_p2_buf[0] = UCI_DIR_BYTE_READ;

    ret = nucleo_spi_read_both(s_tx_p2_buf, s_rx_p2_buf, phase2_len, UCI_SPI_TIMEOUT_MS);
    if (ret != 0) {
        return UCI_STATUS_TRANSPORT_ERROR;
    }

    /* Copy payload into output buffer after the header (skip direction byte echo) */
    memcpy(&buffer[UCI_HDR_SIZE], &s_rx_p2_buf[1], payload_len);

    *out_len = UCI_HDR_SIZE + payload_len;
    return UCI_STATUS_OK;
}

/* -------------------------------------------------------------------------- */
/*  INT Line Polling                                                          */
/* -------------------------------------------------------------------------- */

bool uci_transport_data_available(void)
{
    return (nucleo_spi_int_line_get_state() == 1u);
}

/* -------------------------------------------------------------------------- */
/*  Hardware Reset                                                            */
/* -------------------------------------------------------------------------- */

void uci_transport_hw_reset(void)
{
    /* Assert RST_N low (active-low reset) */
    nucleo_nxp_reset_line_assert(true);
    HAL_Delay(UCI_RESET_HOLD_MS);

    /* Release RST_N */
    nucleo_nxp_reset_line_assert(false);

    /* Wait for SR250 to boot into UCI mode (~50ms typical, 100ms with margin) */
    HAL_Delay(UCI_POST_RESET_WAIT_MS);
}
