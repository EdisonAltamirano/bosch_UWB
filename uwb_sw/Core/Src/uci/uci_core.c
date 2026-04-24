/*
 * uci_core.c
 *
 * UCI core engine implementation.
 *
 * Provides:
 *   - Packet header construction and parsing
 *   - Synchronous command/response with timeout (polling-based, no RTOS)
 *   - PBF-based fragmentation and reassembly
 *   - Notification dispatch to registered callback
 *   - Single-command window (one outstanding command at a time per UCI spec)
 */

#include "uci/uci_core.h"
#include "uci/uci_transport.h"
#include "uci/uci_defs.h"
#include "main.h"
#include <string.h>

/* ========================================================================== */
/*  Internal State                                                            */
/* ========================================================================== */

static struct {
    /* Pending command tracking (for response validation) */
    uint8_t  pending_gid;
    uint8_t  pending_oid;
    bool     response_pending;

    /* Response storage */
    uint8_t  rsp_buffer[UCI_MAX_RSP_BUF_SIZE];
    uint16_t rsp_total_len;          /* Header + payload bytes in rsp_buffer */
    bool     response_received;

    /* Notification callback */
    uci_ntf_callback_t ntf_callback;

    /* Fragmentation reassembly */
    uint8_t  reasm_buffer[UCI_MAX_RSP_BUF_SIZE];
    uint16_t reasm_payload_len;
    uint8_t  reasm_gid;
    uint8_t  reasm_oid;
    uint8_t  reasm_mt;
    bool     reasm_active;

    /* Device state (updated from CORE_DEVICE_STATUS_NTF) */
    uint8_t  device_state;

    /* Last sent command packet (for retry on UCI_STATUS_MESSAGE_RETRY) */
    uint8_t  last_cmd[UCI_MAX_CMD_BUF_SIZE];
    uint16_t last_cmd_len;
} s_ctx;

/* ========================================================================== */
/*  Header Construction / Parsing                                             */
/* ========================================================================== */

void uci_build_header(uint8_t *buf, uint8_t mt, uint8_t pbf,
                       uint8_t gid, uint8_t oid, uint16_t payload_len)
{
    /* Octet 0: MT[7:5] | PBF[4] | GID[3:0] */
    buf[0] = (uint8_t)(((mt & 0x07) << 5) | ((pbf & 0x01) << 4) | (gid & 0x0F));

    if (payload_len > UCI_MAX_PAYLOAD_SIZE) {
        /* NXP extended length: set EXT bit in octet 1, 16-bit length in octets 2-3 */
        buf[1] = (uint8_t)((oid & 0x3F) | UCI_EXT_LEN_BIT);
        buf[2] = (uint8_t)(payload_len >> 8);
        buf[3] = (uint8_t)(payload_len & 0xFF);
    } else {
        /* Standard: OID in octet 1, RFU=0 in octet 2, 8-bit length in octet 3 */
        buf[1] = (uint8_t)(oid & 0x3F);
        buf[2] = 0x00;
        buf[3] = (uint8_t)(payload_len & 0xFF);
    }
}

void uci_parse_header(const uint8_t *buf, uci_header_t *hdr)
{
    hdr->mt  = (buf[0] >> 5) & 0x07;
    hdr->pbf = (buf[0] >> 4) & 0x01;
    hdr->gid = buf[0] & 0x0F;
    hdr->oid = buf[1] & 0x3F;

    if (buf[1] & UCI_EXT_LEN_BIT) {
        hdr->payload_len = ((uint16_t)buf[3] << 8) | (uint16_t)buf[2];
    } else {
        hdr->payload_len = (uint16_t)buf[3];
    }
}

/* ========================================================================== */
/*  Internal Helpers                                                          */
/* ========================================================================== */

/* Dispatch a fully reassembled or non-fragmented packet */
static void uci_dispatch_packet(uint8_t mt, uint8_t gid, uint8_t oid,
                                 const uint8_t *payload, uint16_t payload_len)
{
    switch (mt) {
    case UCI_MT_RSP:
        /* Check if this response matches our pending command */
        if (s_ctx.response_pending &&
            s_ctx.pending_gid == gid &&
            s_ctx.pending_oid == oid) {

            /* Check for retry request (status byte is payload[0]) */
            if (payload_len > 0 && payload[0] == UCI_STATUS_MESSAGE_RETRY) {
                /* Retransmit the last command */
                if (s_ctx.last_cmd_len > 0) {
                    uci_transport_send(s_ctx.last_cmd, s_ctx.last_cmd_len);
                }
                return;
            }

            /* Store the response */
            uci_build_header(s_ctx.rsp_buffer, mt, UCI_PBF_COMPLETE,
                             gid, oid, payload_len);
            if (payload_len > 0 && payload_len <= (UCI_MAX_RSP_BUF_SIZE - UCI_HDR_SIZE)) {
                memcpy(&s_ctx.rsp_buffer[UCI_HDR_SIZE], payload, payload_len);
            }
            s_ctx.rsp_total_len = UCI_HDR_SIZE + payload_len;
            s_ctx.response_received = true;
        }
        break;

    case UCI_MT_NTF:
        /* Handle internal notifications */
        if (gid == UCI_GID_CORE && oid == UCI_OID_CORE_DEVICE_STATUS_NTF) {
            if (payload_len >= 1) {
                s_ctx.device_state = payload[0];
            }
        }

        if (s_ctx.response_pending) {

            /* Check for retry request (status byte is payload[0]) */
            if (payload_len > 0 && payload[0] == UCI_STATUS_MESSAGE_RETRY) {
                /* Retransmit the last command */
                if (s_ctx.last_cmd_len > 0) {
                    uci_transport_send(s_ctx.last_cmd, s_ctx.last_cmd_len);
                }
                return;
            }
        }

        /* Forward all notifications to application callback */
        if (s_ctx.ntf_callback != NULL) {
        	if (gid == UCI_GID_VENDOR_NXP_PROP && oid == UCI_OID_RADAR_RX_NTF) {
        		s_ctx.ntf_callback(gid, oid, payload, payload_len);
        	}
        	else {
        		s_ctx.ntf_callback(gid, oid, payload, payload_len);
        	}
        }
        break;

    default:
        /* UCI_MT_DATA or unknown — forward to callback if registered */
        if (s_ctx.ntf_callback != NULL) {
            s_ctx.ntf_callback(gid, oid, payload, payload_len);
        }
        break;
    }
}

/* Process a single received raw packet (handles fragmentation) */
static void uci_process_raw_packet(const uint8_t *raw, uint16_t raw_len)
{
    if (raw_len < UCI_HDR_SIZE) {
        return;
    }

    uci_header_t hdr;
    uci_parse_header(raw, &hdr);

    const uint8_t *payload = &raw[UCI_HDR_SIZE];
    uint16_t payload_len = hdr.payload_len;

    /* Sanity check: payload_len should not exceed what we actually received */
    if (payload_len > (raw_len - UCI_HDR_SIZE)) {
        payload_len = raw_len - UCI_HDR_SIZE;
    }

    if (hdr.pbf == UCI_PBF_SEGMENT) {
        /* This is a fragment — accumulate in reassembly buffer */
        if (!s_ctx.reasm_active) {
            /* First fragment */
            s_ctx.reasm_active = true;
            s_ctx.reasm_mt  = hdr.mt;
            s_ctx.reasm_gid = hdr.gid;
            s_ctx.reasm_oid = hdr.oid;
            s_ctx.reasm_payload_len = 0;
        }

        /* Append payload to reassembly buffer (bounds check) */
        if ((s_ctx.reasm_payload_len + payload_len) <=
            (UCI_MAX_RSP_BUF_SIZE - UCI_HDR_SIZE)) {
            memcpy(&s_ctx.reasm_buffer[UCI_HDR_SIZE + s_ctx.reasm_payload_len],
                   payload, payload_len);
            s_ctx.reasm_payload_len += payload_len;
        }
        /* else: overflow — drop fragment silently */

    } else {
        /* PBF == COMPLETE */
        if (s_ctx.reasm_active) {
            /* Final fragment of a multi-segment packet */
            if ((s_ctx.reasm_payload_len + payload_len) <=
                (UCI_MAX_RSP_BUF_SIZE - UCI_HDR_SIZE)) {
                memcpy(&s_ctx.reasm_buffer[UCI_HDR_SIZE + s_ctx.reasm_payload_len],
                       payload, payload_len);
                s_ctx.reasm_payload_len += payload_len;
            }

            /* Dispatch the fully reassembled packet */
            uci_dispatch_packet(s_ctx.reasm_mt, s_ctx.reasm_gid, s_ctx.reasm_oid,
                                &s_ctx.reasm_buffer[UCI_HDR_SIZE],
                                s_ctx.reasm_payload_len);

            /* Reset reassembly state */
            s_ctx.reasm_active = false;
            s_ctx.reasm_payload_len = 0;

        } else {
            /* Non-fragmented packet — dispatch directly */
            uci_dispatch_packet(hdr.mt, hdr.gid, hdr.oid, payload, payload_len);
        }
    }
}

/* ========================================================================== */
/*  Public API                                                                */
/* ========================================================================== */

void uci_core_init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.device_state = UCI_DEVICE_STATUS_ERROR; /* Unknown until first NTF */
}

void uci_core_register_ntf_callback(uci_ntf_callback_t callback)
{
    s_ctx.ntf_callback = callback;
}

void uci_core_process(void)
{
    if (!uci_transport_data_available()) {
        return;
    }

    /* Read one packet from SR250 */
    static uint8_t rx_buf[UCI_MAX_RSP_BUF_SIZE];
    uint16_t rx_len = 0;
    uint16_t p_ctr = 0;

    uci_status_t st = uci_transport_receive(rx_buf, &rx_len, sizeof(rx_buf));
    if (st != UCI_STATUS_OK || rx_len < UCI_HDR_SIZE) {
        return;
    }

    uci_process_raw_packet(rx_buf, rx_len);
}

uci_status_t uci_core_send_command(
    uint8_t gid, uint8_t oid,
    const uint8_t *payload, uint16_t payload_len,
    uint8_t *rsp_payload, uint16_t *rsp_len,
    uint32_t timeout_ms)
{
    static uint8_t cmd_buf[UCI_MAX_CMD_BUF_SIZE];
    uci_status_t st;

    /* ── Build command packet, handling fragmentation if needed ── */
    if (payload_len <= UCI_MAX_PAYLOAD_SIZE) {
        /* Single packet (no fragmentation) */
        uci_build_header(cmd_buf, UCI_MT_CMD, UCI_PBF_COMPLETE,
                         gid, oid, payload_len);
        if (payload != NULL && payload_len > 0) {
            memcpy(&cmd_buf[UCI_HDR_SIZE], payload, payload_len);
        }
        uint16_t total_len = UCI_HDR_SIZE + payload_len;

        /* Save for potential retry */
        memcpy(s_ctx.last_cmd, cmd_buf, total_len);
        s_ctx.last_cmd_len = total_len;

        /* Set up response tracking */
        s_ctx.pending_gid = gid;
        s_ctx.pending_oid = oid;
        s_ctx.response_pending = true;
        s_ctx.response_received = false;

        /* Send */
        st = uci_transport_send(cmd_buf, total_len);
        if (st != UCI_STATUS_OK) {
            s_ctx.response_pending = false;
            return UCI_STATUS_TRANSPORT_ERROR;
        }
    } else {
        /* Fragmented send: split payload into 255-byte chunks */
        uint16_t offset = 0;
        uint16_t remaining = payload_len;

        while (remaining > 0) {
            uint16_t chunk = (remaining > UCI_MAX_PAYLOAD_SIZE)
                             ? UCI_MAX_PAYLOAD_SIZE : remaining;
            uint8_t pbf = (remaining > chunk) ? UCI_PBF_SEGMENT : UCI_PBF_COMPLETE;

            uci_build_header(cmd_buf, UCI_MT_CMD, pbf, gid, oid, chunk);
            if (payload != NULL) {
                memcpy(&cmd_buf[UCI_HDR_SIZE], &payload[offset], chunk);
            }

            uint16_t total_len = UCI_HDR_SIZE + chunk;

            /* For the last fragment, save for retry and set up response tracking */
            if (pbf == UCI_PBF_COMPLETE) {
                memcpy(s_ctx.last_cmd, cmd_buf, total_len);
                s_ctx.last_cmd_len = total_len;
                s_ctx.pending_gid = gid;
                s_ctx.pending_oid = oid;
                s_ctx.response_pending = true;
                s_ctx.response_received = false;
            }

            st = uci_transport_send(cmd_buf, total_len);
            if (st != UCI_STATUS_OK) {
                s_ctx.response_pending = false;
                return UCI_STATUS_TRANSPORT_ERROR;
            }

            offset += chunk;
            remaining -= chunk;
        }
    }

    /* ── Poll for response ── */
    uint32_t start_tick = HAL_GetTick();

    while (!s_ctx.response_received) {
        uci_core_process();

        uint32_t elapsed = HAL_GetTick() - start_tick;
        if (elapsed >= timeout_ms) {
            s_ctx.response_pending = false;
            return UCI_STATUS_TIMEOUT;
        }
    }

    s_ctx.response_pending = false;

    /* ── Extract response ── */
    uci_header_t rsp_hdr;
    uci_parse_header(s_ctx.rsp_buffer, &rsp_hdr);

    uint16_t rsp_payload_len = rsp_hdr.payload_len;
    const uint8_t *rsp_data = &s_ctx.rsp_buffer[UCI_HDR_SIZE];

    /* Copy response payload to caller's buffer */
    if (rsp_payload != NULL && rsp_len != NULL) {
        uint16_t copy_len = rsp_payload_len;
        if (copy_len > *rsp_len) {
            copy_len = *rsp_len;
        }
        if (copy_len > 0) {
            memcpy(rsp_payload, rsp_data, copy_len);
        }
        *rsp_len = copy_len;
    } else if (rsp_len != NULL) {
        *rsp_len = rsp_payload_len;
    }

    /* Return the status byte (first byte of response payload), or OK if empty */
    if (rsp_payload_len > 0) {
        return rsp_data[0];
    }
    return UCI_STATUS_OK;
}

uint8_t uci_core_get_device_state(void)
{
    return s_ctx.device_state;
}
