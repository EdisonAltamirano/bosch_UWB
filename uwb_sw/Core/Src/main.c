/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "udp_server.h"
#include <string.h>
#include "uci/uci_core.h"
#include "uci/uci_commands.h"
#include "uci/uci_radar.h"
#include "uci/uci_sr250.h"
#include "uci/uci_transport.h"
#include "uwb_udp_protocol.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
__IO uint32_t BspButtonState = BUTTON_RELEASED;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern struct netif gnetif;

/* ── Notification handler (forward declaration) ── */
static void app_uci_notification_handler(uint8_t gid, uint8_t oid,
                                          const uint8_t *payload, uint16_t len);

/* ── Radar session state ── */
/* ── Board-specific antenna configuration for Truesense ETNA ──
 * NOTE: These antenna IDs and port mappings must match the ETNA board schematic.
 *       Update the values below to match your specific hardware wiring. */
static const uci_rx_antenna_def_t etna_rx_antennas[] = {
	{ .antenna_id = 0x01, .rx_port = NXP_RX_PORT_RXC,  .gpio_mask_lsb = 0x00, .gpio_mask_msb = 0x00, .gpio_state_lsb = 0x00, .gpio_state_msb = 0x00 },
	{ .antenna_id = 0x02, .rx_port = NXP_RX_PORT_RXB, .gpio_mask_lsb = 0x00, .gpio_mask_msb = 0x00, .gpio_state_lsb = 0x00, .gpio_state_msb = 0x00 },
    { .antenna_id = 0x03, .rx_port = NXP_RX_PORT_RXA2,  .gpio_mask_lsb = 0x00, .gpio_mask_msb = 0x00, .gpio_state_lsb = 0x00, .gpio_state_msb = 0x00 },
    { .antenna_id = 0x04, .rx_port = NXP_RX_PORT_RXA1, .gpio_mask_lsb = 0x00, .gpio_mask_msb = 0x00, .gpio_state_lsb = 0x00, .gpio_state_msb = 0x00 }
};
#define ETNA_NUM_RX_ANT  (sizeof(etna_rx_antennas) / sizeof(etna_rx_antennas[0]))

static const uci_tx_antenna_def_t etna_tx_antennas[] = {
    { .antenna_id = 0x01, .tx_port = NXP_TX_PORT_TRA1, .gpio_mask_lsb = 0x00, .gpio_mask_msb = 0x00, .gpio_state_lsb = 0x00, .gpio_state_msb = 0x00 },
	{ .antenna_id = 0x02, .tx_port = NXP_TX_PORT_TRA2, .gpio_mask_lsb = 0x00, .gpio_mask_msb = 0x00, .gpio_state_lsb = 0x00, .gpio_state_msb = 0x00 }
};
#define ETNA_NUM_TX_ANT  (sizeof(etna_tx_antennas) / sizeof(etna_tx_antennas[0]))

static const uci_rx_antenna_pair_t etna_rx_pairs[] = {
    { .pair_id = 0x01, .antenna_rxc = 0x01, .antenna_rxb = 0x02, .antenna_rxa = 0x00, .rfu_lsb = 0x00, .rfu_msb = 0x00 },
	{ .pair_id = 0x02, .antenna_rxc = 0x00, .antenna_rxb = 0x02, .antenna_rxa = 0x03, .rfu_lsb = 0x00, .rfu_msb = 0x00 }
};
#define ETNA_NUM_RX_PAIRS (sizeof(etna_rx_pairs) / sizeof(etna_rx_pairs[0]))

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LWIP_Init();
  MX_SPI6_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN BSP */

  /* -- Sample board code to send message over COM1 port ---- */
  printf("Welcome to STM32 world !\n\r");

  /* -- Sample board code to switch on leds ---- */
  BSP_LED_On(LED_YELLOW);
  BSP_LED_On(LED_RED);

  nucleo_udp_init();

  /* ── Initialize UCI stack ── */
  uci_transport_init();
  uci_core_init();
  uwb_udp_protocol_init();
  uci_core_register_ntf_callback(app_uci_notification_handler);

  printf("UCI stack initialized\n\r");

  /* ── Full SR250 initialization: reset → device_init → antenna config → calibration ── */
  uci_sr250_init_config_t init_cfg = {
      .rx_antennas        = etna_rx_antennas,
      .num_rx_antennas    = ETNA_NUM_RX_ANT,
      .tx_antennas        = etna_tx_antennas,
      .num_tx_antennas    = ETNA_NUM_TX_ANT,
      .rx_pairs           = etna_rx_pairs,
      .num_rx_pairs       = ETNA_NUM_RX_PAIRS,
      .run_chip_calibration = true,
      .channel            = 9,
  };

  uci_status_t init_st = uci_sr250_full_init(&init_cfg);
  if (init_st != UCI_STATUS_OK) {
      printf("SR250 init failed: 0x%02X\n\r", init_st);
      BSP_LED_Off(LED_YELLOW);
  } else {
      printf("SR250 init OK\n\r");
  }

  /* ── Start a radar session with presence detection + AoA ── */
  /* USER CODE END BSP */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Process any pending UCI data from SR250 (dispatches RADAR_RX_NTF to callback) */
    uci_core_process();

    /* Process Ethernet / LwIP */
    MX_LWIP_Process();



  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 120;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ── UCI Notification Handler ──
 * Called from uci_core_process() whenever a notification arrives from SR250.
 * Handles RADAR_RX_NTF (CIR, presence, antenna isolation) and session status. */
static void app_uci_notification_handler(uint8_t gid, uint8_t oid,
                                          const uint8_t *payload, uint16_t len)
{
    if (gid == UCI_GID_VENDOR_NXP_PROP && oid == UCI_OID_RADAR_RX_NTF) {
        /* RADAR_RX_NTF — identify data type and parse */
        uci_radar_data_type_t data_type = uci_radar_get_ntf_type(payload, len);

        switch (data_type) {
        case UCI_RADAR_DATA_CIR_SAMPLES: {
            uci_radar_cir_metadata_t meta;
            const uint8_t *cir_data;
            uint16_t num_taps;
            uci_radar_parse_cir_ntf(payload, len, &meta, &cir_data, &num_taps);
            uwb_udp_protocol_send_radar_frame(payload, len);
            printf("CIR: rx=%d taps=%d offset=%d\n\r",
                   meta.rx_path, num_taps, meta.cir_start_offset);
            break;
        }
        case UCI_RADAR_DATA_PRESENCE: {
            uci_radar_presence_ntf_t presence;
            uci_radar_parse_presence_ntf(payload, len, &presence);
            uwb_udp_protocol_send_radar_frame(payload, len);
            if (presence.presence_detected) {
                for (uint8_t i = 0; i < presence.num_detections; i++) {
                    printf("Target %d: dist=%dcm angle=%d deg\n\r", i,
                           presence.targets[i].distance_cm,
                           presence.targets[i].angle_deg);
                }
            } else {
                printf("No presence\n\r");
            }
            break;
        }
        case UCI_RADAR_DATA_ANT_ISOLATION: {
            uci_radar_ant_isolation_ntf_t iso;
            uci_radar_parse_ant_isolation_ntf(payload, len, &iso);
            uwb_udp_protocol_send_radar_frame(payload, len);
            printf("Isolation: TX%d->RX%d = %ddB\n\r",
                   iso.tx_antenna_id, iso.rx_antenna_id, iso.isolation_db);
            break;
        }
        default:
            break;
        }
    }
    else if (gid == UCI_GID_CORE && oid == UCI_OID_CORE_DEVICE_STATUS_NTF) {
        if (len >= 1) {
            printf("Device state: 0x%02X\n\r", payload[0]);
        }
    }
    else if (gid == UCI_GID_SESSION_CONFIG && oid == UCI_OID_SESSION_STATUS_NTF) {
        if (len >= 6) {
            uint8_t session_state = payload[4];
            uint8_t reason = payload[5];
            printf("Session state: 0x%02X reason: 0x%02X\n\r", session_state, reason);
        }
    }
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_128KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
