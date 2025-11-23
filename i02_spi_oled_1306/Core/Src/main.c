/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 解析度設定 */
#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT   64
/* 連接腳位（依你的接線調整） */
#define SSD1306_SPI_HANDLE   hspi1
extern SPI_HandleTypeDef SSD1306_SPI_HANDLE;
#define SSD1306_DC_Port   GPIOD
#define SSD1306_DC_Pin    GPIO_PIN_12
#define SSD1306_RST_Port  GPIOD
#define SSD1306_RST_Pin   GPIO_PIN_13
#define SSD1306_CS_Port   GPIOD
#define SSD1306_CS_Pin    GPIO_PIN_11
/* 公用 API */
void     SSD1306_Init(void);
void     SSD1306_Update(void);
void     SSD1306_Fill(uint8_t color);    /* 0x00=黑, 0xFF=白(每頁位元) */
void     SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color);
/* 簡易工具 */
static inline void SSD1306_Select(void)   { HAL_GPIO_WritePin(SSD1306_CS_Port,  SSD1306_CS_Pin,  GPIO_PIN_RESET); }
static inline void SSD1306_Unselect(void) { HAL_GPIO_WritePin(SSD1306_CS_Port,  SSD1306_CS_Pin,  GPIO_PIN_SET); }
static inline void SSD1306_DC_Command(void){ HAL_GPIO_WritePin(SSD1306_DC_Port, SSD1306_DC_Pin, GPIO_PIN_RESET); }
static inline void SSD1306_DC_Data(void)   { HAL_GPIO_WritePin(SSD1306_DC_Port, SSD1306_DC_Pin, GPIO_PIN_SET);   }
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
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart3;
PCD_HandleTypeDef hpcd_USB_OTG_FS;
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
 /* USER CODE BEGIN 1 */
 /* USER CODE END 1 */
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
 MX_USART3_UART_Init();
 MX_USB_OTG_FS_PCD_Init();
 MX_SPI1_Init();
 /* USER CODE BEGIN 2 */
 /* USER CODE END 2 */
 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
 SSD1306_Init();                // 啟動顯示器
 /* Demo：畫斜線與邊框，再全螢幕更新 */
   SSD1306_Fill(0x00);            // 清黑
   for (uint16_t i = 0; i < 64; ++i) {
       SSD1306_DrawPixel(i*2, i, 1);            // 斜線
   }
   for (uint16_t x = 0; x < SSD1306_WIDTH;  ++x) { // 上/下邊框
       SSD1306_DrawPixel(x, 0, 1);
       SSD1306_DrawPixel(x, SSD1306_HEIGHT-1, 1);
   }
   for (uint16_t y = 0; y < SSD1306_HEIGHT; ++y) { // 左/右邊框
       SSD1306_DrawPixel(0, y, 1);
       SSD1306_DrawPixel(SSD1306_WIDTH-1, y, 1);
   }
   SSD1306_Update();
 while (1)
 {
	  HAL_Delay(1000);
   /* USER CODE END WHILE */
   /* USER CODE BEGIN 3 */
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
 /** Configure the main internal regulator output voltage
 */
 __HAL_RCC_PWR_CLK_ENABLE();
 __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
 /** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLM = 8;
 RCC_OscInitStruct.PLL.PLLN = 384;
 RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
 RCC_OscInitStruct.PLL.PLLQ = 8;
 RCC_OscInitStruct.PLL.PLLR = 2;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
 {
   Error_Handler();
 }
 /** Initializes the CPU, AHB and APB buses clocks
 */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                             |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
 {
   Error_Handler();
 }
}
/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{
 /* USER CODE BEGIN SPI1_Init 0 */
 /* USER CODE END SPI1_Init 0 */
 /* USER CODE BEGIN SPI1_Init 1 */
 /* USER CODE END SPI1_Init 1 */
 /* SPI1 parameter configuration*/
 hspi1.Instance = SPI1;
 hspi1.Init.Mode = SPI_MODE_MASTER;
 hspi1.Init.Direction = SPI_DIRECTION_2LINES;
 hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
 hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
 hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
 hspi1.Init.NSS = SPI_NSS_SOFT;
 hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
 hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
 hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
 hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
 hspi1.Init.CRCPolynomial = 7;
 if (HAL_SPI_Init(&hspi1) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN SPI1_Init 2 */
 /* USER CODE END SPI1_Init 2 */
}
/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
 /* USER CODE BEGIN USART3_Init 0 */
 /* USER CODE END USART3_Init 0 */
 /* USER CODE BEGIN USART3_Init 1 */
 /* USER CODE END USART3_Init 1 */
 huart3.Instance = USART3;
 huart3.Init.BaudRate = 115200;
 huart3.Init.WordLength = UART_WORDLENGTH_8B;
 huart3.Init.StopBits = UART_STOPBITS_1;
 huart3.Init.Parity = UART_PARITY_NONE;
 huart3.Init.Mode = UART_MODE_TX_RX;
 huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
 huart3.Init.OverSampling = UART_OVERSAMPLING_16;
 if (HAL_UART_Init(&huart3) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN USART3_Init 2 */
 /* USER CODE END USART3_Init 2 */
}
/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void)
{
 /* USER CODE BEGIN USB_OTG_FS_Init 0 */
 /* USER CODE END USB_OTG_FS_Init 0 */
 /* USER CODE BEGIN USB_OTG_FS_Init 1 */
 /* USER CODE END USB_OTG_FS_Init 1 */
 hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
 hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
 hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
 hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
 hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
 hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
 hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
 hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
 hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
 hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
 hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
 if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
 {
   Error_Handler();
 }
 /* USER CODE BEGIN USB_OTG_FS_Init 2 */
 /* USER CODE END USB_OTG_FS_Init 2 */
}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};
 /* USER CODE BEGIN MX_GPIO_Init_1 */
 /* USER CODE END MX_GPIO_Init_1 */
 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOC_CLK_ENABLE();
 __HAL_RCC_GPIOH_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();
 __HAL_RCC_GPIOD_CLK_ENABLE();
 __HAL_RCC_GPIOG_CLK_ENABLE();
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);
 /*Configure GPIO pin : USER_Btn_Pin */
 GPIO_InitStruct.Pin = USER_Btn_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);
 /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
 GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
 /*Configure GPIO pins : PD11 PD12 PD13 */
 GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
 /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
 GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
 HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);
 /*Configure GPIO pin : USB_OverCurrent_Pin */
 GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull = GPIO_NOPULL;
 HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);
 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
/* 畫面緩衝：128x64/8 = 1024 Bytes */
static uint8_t ssd1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];
static void ssd1306_Reset(void) {
   HAL_GPIO_WritePin(SSD1306_RST_Port, SSD1306_RST_Pin, GPIO_PIN_RESET);
   HAL_Delay(5);
   HAL_GPIO_WritePin(SSD1306_RST_Port, SSD1306_RST_Pin, GPIO_PIN_SET);
   HAL_Delay(5);
}
static void ssd1306_WriteCommand(uint8_t cmd) {
   SSD1306_Select();
   SSD1306_DC_Command();
   HAL_SPI_Transmit(&SSD1306_SPI_HANDLE, &cmd, 1, HAL_MAX_DELAY);
   SSD1306_Unselect();
}
static void ssd1306_WriteData(uint8_t *data, size_t size) {
   SSD1306_Select();
   SSD1306_DC_Data();
   HAL_SPI_Transmit(&SSD1306_SPI_HANDLE, data, size, HAL_MAX_DELAY);
   SSD1306_Unselect();
}
void SSD1306_Init(void) {
   /* 硬體復位 */
   ssd1306_Reset();
   /* 初始化命令序列（128x64 常用設定） */
   ssd1306_WriteCommand(0xAE);         // display off
   ssd1306_WriteCommand(0xD5); ssd1306_WriteCommand(0x80); // clock divide / oscillator
   ssd1306_WriteCommand(0xA8); ssd1306_WriteCommand(0x3F); // multiplex 1/64
   ssd1306_WriteCommand(0xD3); ssd1306_WriteCommand(0x00); // display offset
   ssd1306_WriteCommand(0x40);         // start line = 0
   ssd1306_WriteCommand(0x8D); ssd1306_WriteCommand(0x14); // charge pump on (internal)
   ssd1306_WriteCommand(0x20); ssd1306_WriteCommand(0x00); // memory address mode: horizontal
   ssd1306_WriteCommand(0xA1);         // segment remap
   ssd1306_WriteCommand(0xC8);         // COM scan direction remap
   ssd1306_WriteCommand(0xDA); ssd1306_WriteCommand(0x12); // COM pins config for 128x64
   ssd1306_WriteCommand(0x81); ssd1306_WriteCommand(0x7F); // contrast
   ssd1306_WriteCommand(0xD9); ssd1306_WriteCommand(0xF1); // pre-charge
   ssd1306_WriteCommand(0xDB); ssd1306_WriteCommand(0x40); // VCOMH deselect level
   ssd1306_WriteCommand(0xA4);         // display follows RAM
   ssd1306_WriteCommand(0xA6);         // normal display (not inverted)
   ssd1306_WriteCommand(0xAF);         // display on
   /* 清畫面緩衝 */
   SSD1306_Fill(0x00);
   SSD1306_Update();
}
void SSD1306_Fill(uint8_t color) {
   memset(ssd1306_Buffer, color ? 0xFF : 0x00, sizeof(ssd1306_Buffer));
}
/* 單點繪製；y 由 0..63，對應到某頁與頁內位元 */
void SSD1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color) {
   if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;
   uint32_t index = x + (y / 8) * SSD1306_WIDTH;
   uint8_t  mask  = 1 << (y % 8);
   if (color) ssd1306_Buffer[index] |= mask;
   else       ssd1306_Buffer[index] &= ~mask;
}
/* 將整個緩衝以水平位址模式一次刷到 0..127 × 0..7 頁 */
void SSD1306_Update(void) {
   /* 設定列範圍（Column）與頁範圍（Page） */
   ssd1306_WriteCommand(0x21); // Set Column Address
   ssd1306_WriteCommand(0x00);
   ssd1306_WriteCommand(SSD1306_WIDTH - 1);
   ssd1306_WriteCommand(0x22); // Set Page Address
   ssd1306_WriteCommand(0x00);
   ssd1306_WriteCommand((SSD1306_HEIGHT / 8) - 1);
   /* 送出整個 frame buffer */
   ssd1306_WriteData(ssd1306_Buffer, sizeof(ssd1306_Buffer));
}
/* USER CODE END 4 */
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
