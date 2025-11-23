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
#include "main.h" // 引入主要硬體定義標頭檔
#include "cmsis_os.h" // 引入 CMSIS-RTOS API 標頭檔
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
UART_HandleTypeDef huart3; // 定義 UART3 的控制代碼 (Handle)
PCD_HandleTypeDef hpcd_USB_OTG_FS; // 定義 USB OTG FS 的控制代碼
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle; // 定義 defaultTask 的執行緒 ID 變數
const osThreadAttr_t defaultTask_attributes = { // 定義 defaultTask 的屬性結構
 .name = "defaultTask", // 任務名稱
 .stack_size = 128 * 4, // 堆疊大小 (以 byte 為單位，這裡約 512 bytes)
 .priority = (osPriority_t) osPriorityNormal, // 任務優先級 (普通)
};
/* Definitions for Led1Task */
osThreadId_t Led1TaskHandle; // 定義 Led1Task 的執行緒 ID 變數
const osThreadAttr_t Led1Task_attributes = { // 定義 Led1Task 的屬性
 .name = "Led1Task", // 任務名稱
 .stack_size = 128 * 4, // 堆疊大小
 .priority = (osPriority_t) osPriorityNormal, // 任務優先級 (普通)
};
/* Definitions for Led2Task */
osThreadId_t Led2TaskHandle; // 定義 Led2Task 的執行緒 ID 變數
const osThreadAttr_t Led2Task_attributes = { // 定義 Led2Task 的屬性
 .name = "Led2Task", // 任務名稱
 .stack_size = 128 * 4, // 堆疊大小
 .priority = (osPriority_t) osPriorityNormal, // 任務優先級 (普通)
};
/* Definitions for Led3Task */
osThreadId_t Led3TaskHandle; // 定義 Led3Task 的執行緒 ID 變數
const osThreadAttr_t Led3Task_attributes = { // 定義 Led3Task 的屬性
 .name = "Led3Task", // 任務名稱
 .stack_size = 128 * 4, // 堆疊大小
 .priority = (osPriority_t) osPriorityLow, // 任務優先級 (低，比 Normal 慢執行)
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void); // 系統時鐘設定函式宣告
static void MX_GPIO_Init(void); // GPIO 初始化函式宣告
static void MX_USART3_UART_Init(void); // UART3 初始化函式宣告
static void MX_USB_OTG_FS_PCD_Init(void); // USB 初始化函式宣告
void StartDefaultTask(void *argument); // DefaultTask 任務主體函式宣告
void _Led1Task(void *argument); // Led1Task 任務主體函式宣告
void _Led2Task(void *argument); // Led2Task 任務主體函式宣告
void _Led3Task(void *argument); // Led3Task 任務主體函式宣告
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) // 主程式入口
{
 /* USER CODE BEGIN 1 */
 /* USER CODE END 1 */
 /* MCU Configuration--------------------------------------------------------*/
 /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
 HAL_Init(); // 初始化 HAL 庫 (重置周邊、Flash 介面、Systick 計時器)
 /* USER CODE BEGIN Init */
 /* USER CODE END Init */
 /* Configure the system clock */
 SystemClock_Config(); // 設定系統時鐘 (RCC)
 /* USER CODE BEGIN SysInit */
 /* USER CODE END SysInit */
 /* Initialize all configured peripherals */
 MX_GPIO_Init(); // 初始化 GPIO (按鈕、LED 等)
 MX_USART3_UART_Init(); // 初始化 UART3 (序列埠通訊)
 MX_USB_OTG_FS_PCD_Init(); // 初始化 USB OTG
 /* USER CODE BEGIN 2 */
 /* USER CODE END 2 */
 /* Init scheduler */
 osKernelInitialize(); // 初始化 FreeRTOS 核心 (準備排程器)
 /* USER CODE BEGIN RTOS_MUTEX */
 /* add mutexes, ... */
 /* USER CODE END RTOS_MUTEX */
 /* USER CODE BEGIN RTOS_SEMAPHORES */
 /* add semaphores, ... */
 /* USER CODE END RTOS_SEMAPHORES */
 /* USER CODE BEGIN RTOS_TIMERS */
 /* start timers, add new ones, ... */
 /* USER CODE END RTOS_TIMERS */
 /* USER CODE BEGIN RTOS_QUEUES */
 /* add queues, ... */
 /* USER CODE END RTOS_QUEUES */
 /* Create the thread(s) */
 /* creation of defaultTask */
 defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes); // 建立 defaultTask 執行緒
 /* creation of Led1Task */
 Led1TaskHandle = osThreadNew(_Led1Task, NULL, &Led1Task_attributes); // 建立 Led1Task 執行緒
 /* creation of Led2Task */
 Led2TaskHandle = osThreadNew(_Led2Task, NULL, &Led2Task_attributes); // 建立 Led2Task 執行緒
 /* creation of Led3Task */
 Led3TaskHandle = osThreadNew(_Led3Task, NULL, &Led3Task_attributes); // 建立 Led3Task 執行緒
 /* USER CODE BEGIN RTOS_THREADS */
 /* add threads, ... */
 /* USER CODE END RTOS_THREADS */
 /* USER CODE BEGIN RTOS_EVENTS */
 /* add events, ... */
 /* USER CODE END RTOS_EVENTS */
 /* Start scheduler */
 osKernelStart(); // 啟動 RTOS 排程器 (從這裡開始，程式控制權交給 FreeRTOS，不再往下執行)
 /* We should never get here as control is now taken by the scheduler */
 /* Infinite loop */
 /* USER CODE BEGIN WHILE */
 while (1) // 無窮迴圈 (理論上永遠不會執行到這，除非 OS 啟動失敗)
 {
   /* USER CODE END WHILE */
   /* USER CODE BEGIN 3 */
 }
 /* USER CODE END 3 */
}
/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) // 系統時鐘設定實作
{
 RCC_OscInitTypeDef RCC_OscInitStruct = {0}; // 定義振盪器設定結構體
 RCC_ClkInitTypeDef RCC_ClkInitStruct = {0}; // 定義時鐘設定結構體
 /** Configure the main internal regulator output voltage
 */
 __HAL_RCC_PWR_CLK_ENABLE(); // 啟用電源控制介面時鐘
 __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); // 設定電壓調節器為 Scale 1 (高效能模式)
 /** Initializes the RCC Oscillators according to the specified parameters
 * in the RCC_OscInitTypeDef structure.
 */
 RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // 選擇 HSE (外部高速振盪器)
 RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS; // HSE 模式設為 Bypass (使用 ST-Link 提供的時鐘)
 RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // 啟用 PLL (鎖相迴路)
 RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // PLL 訊號來源選 HSE
 RCC_OscInitStruct.PLL.PLLM = 8; // PLL M 分頻係數
 RCC_OscInitStruct.PLL.PLLN = 384; // PLL N 倍頻係數
 RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; // PLL P 分頻係數
 RCC_OscInitStruct.PLL.PLLQ = 8; // PLL Q 分頻係數 (給 USB 用)
 RCC_OscInitStruct.PLL.PLLR = 2; // PLL R 分頻係數
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) // 套用振盪器設定
 {
   Error_Handler(); // 如果設定失敗，進入錯誤處理
 }
 /** Initializes the CPU, AHB and APB buses clocks
 */
 RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                             |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2; // 設定要配置的時鐘類型
 RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // 系統時鐘來源選用 PLL
 RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB 不分頻
 RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // APB1 除以 2
 RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1; // APB2 不分頻
 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) // 套用時鐘設定與 Flash 延遲
 {
   Error_Handler(); // 如果失敗，進入錯誤處理
 }
}
/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) // UART3 初始化實作
{
 /* USER CODE BEGIN USART3_Init 0 */
 /* USER CODE END USART3_Init 0 */
 /* USER CODE BEGIN USART3_Init 1 */
 /* USER CODE END USART3_Init 1 */
 huart3.Instance = USART3; // 指定硬體實例為 USART3
 huart3.Init.BaudRate = 115200; // 設定波特率為 115200 bps
 huart3.Init.WordLength = UART_WORDLENGTH_8B; // 資料長度 8 bits
 huart3.Init.StopBits = UART_STOPBITS_1; // 停止位 1 bit
 huart3.Init.Parity = UART_PARITY_NONE; // 無同位檢查
 huart3.Init.Mode = UART_MODE_TX_RX; // 模式為 收/發 雙向
 huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 無硬體流控
 huart3.Init.OverSampling = UART_OVERSAMPLING_16; // 16倍過採樣
 if (HAL_UART_Init(&huart3) != HAL_OK) // 初始化 UART 並檢查結果
 {
   Error_Handler(); // 失敗則進入錯誤處理
 }
 /* USER CODE BEGIN USART3_Init 2 */
 // 注意：若要使用中斷接收，需在此處或 HAL_UART_MspInit 中啟用 NVIC
 /* USER CODE END USART3_Init 2 */
}
/**
 * @brief USB_OTG_FS Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_OTG_FS_PCD_Init(void) // USB 初始化實作
{
 /* USER CODE BEGIN USB_OTG_FS_Init 0 */
 /* USER CODE END USB_OTG_FS_Init 0 */
 /* USER CODE BEGIN USB_OTG_FS_Init 1 */
 /* USER CODE END USB_OTG_FS_Init 1 */
 hpcd_USB_OTG_FS.Instance = USB_OTG_FS; // 指定為 USB OTG FS
 hpcd_USB_OTG_FS.Init.dev_endpoints = 6; // 設定端點數量
 hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL; // 全速模式 (12Mbps)
 hpcd_USB_OTG_FS.Init.dma_enable = DISABLE; // 停用 DMA
 hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED; // 使用內建 PHY
 hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE; // 啟用 SOF (Start of Frame)
 hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE; // 停用低功耗
 hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE; // 停用 LPM
 hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE; // 啟用電池充電檢測
 hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE; // 啟用 VBUS 感測
 hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE; // 停用專用 EP1
 if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK) // 初始化 USB
 {
   Error_Handler(); // 失敗則進入錯誤處理
 }
 /* USER CODE BEGIN USB_OTG_FS_Init 2 */
 /* USER CODE END USB_OTG_FS_Init 2 */
}
/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) // GPIO 初始化實作
{
 GPIO_InitTypeDef GPIO_InitStruct = {0}; // 定義 GPIO 設定結構體
 /* USER CODE BEGIN MX_GPIO_Init_1 */
 /* USER CODE END MX_GPIO_Init_1 */
 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOC_CLK_ENABLE(); // 啟用 GPIOC 時鐘
 __HAL_RCC_GPIOH_CLK_ENABLE(); // 啟用 GPIOH 時鐘
 __HAL_RCC_GPIOB_CLK_ENABLE(); // 啟用 GPIOB 時鐘
 __HAL_RCC_GPIOD_CLK_ENABLE(); // 啟用 GPIOD 時鐘
 __HAL_RCC_GPIOG_CLK_ENABLE(); // 啟用 GPIOG 時鐘
 __HAL_RCC_GPIOA_CLK_ENABLE(); // 啟用 GPIOA 時鐘
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET); // 預設將 LED 腳位拉低 (滅)
 /*Configure GPIO pin Output Level */
 HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET); // 預設關閉 USB 電源開關
 /*Configure GPIO pin : USER_Btn_Pin */
 GPIO_InitStruct.Pin = USER_Btn_Pin; // 設定 User Button 腳位
 GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // 設定為中斷模式 (上升緣觸發)
 GPIO_InitStruct.Pull = GPIO_NOPULL; // 不使用內建上拉/下拉
 HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct); // 初始化按鈕 GPIO
 /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
 GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin; // 設定 LED1, LED3, LED2 腳位
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽輸出模式
 GPIO_InitStruct.Pull = GPIO_NOPULL; // 不使用上下拉
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 低速模式
 HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); // 初始化 LED GPIO
 /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
 GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin; // 設定 USB 電源開關腳位
 GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // 推挽輸出
 GPIO_InitStruct.Pull = GPIO_NOPULL; // 不使用上下拉
 GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 低速
 HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct); // 初始化 USB 電源 GPIO
 /*Configure GPIO pin : USB_OverCurrent_Pin */
 GPIO_InitStruct.Pin = USB_OverCurrent_Pin; // 設定 USB 過流偵測腳位
 GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // 輸入模式
 GPIO_InitStruct.Pull = GPIO_NOPULL; // 不使用上下拉
 HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct); // 初始化 USB 過流 GPIO
 /* USER CODE BEGIN MX_GPIO_Init_2 */
 /* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)     // 任務函式 (DefaultTask)
{
 /* USER CODE BEGIN 5 */
 /* Infinite loop */
 const char *msg = "Hello World from FreeRTOS\r\n"; // 定義要傳送的字串
 for(;;) // 任務的無窮迴圈
 {
   HAL_UART_Transmit(&huart3,        // 使用 UART3
                   (uint8_t*)msg,     // 傳送的資料指標
                   strlen(msg),       // 資料長度
                   HAL_MAX_DELAY);    // 等待直到傳送完成 (Blocking 模式)
   vTaskDelay(pdMS_TO_TICKS(1000));    // 延遲 1000ms (讓出 CPU 給其他 Task)
	 // osDelay(1);
 }
 /* USER CODE END 5 */
}
/* USER CODE BEGIN Header__Led1Task */
/**
* @brief Function implementing the Led1Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__Led1Task */
void _Led1Task(void *argument) // 任務函式 (Led1Task)
{
 /* USER CODE BEGIN _Led1Task */
 /* Infinite loop */
 for(;;) // 任務無窮迴圈
 {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0); // 切換 LED1 (GPIOB Pin 0) 狀態
	vTaskDelay(pdMS_TO_TICKS(500));        // 延遲 500ms (讓出 CPU)
   // osDelay(1);
 }
 /* USER CODE END _Led1Task */
}
/* USER CODE BEGIN Header__Led2Task */
/**
* @brief Function implementing the Led2Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__Led2Task */
void _Led2Task(void *argument) // 任務函式 (Led2Task)
{
 /* USER CODE BEGIN _Led2Task */
 /* Infinite loop */
 for(;;) // 任務無窮迴圈
 {
     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7); // 切換 LED2 (GPIOB Pin 7) 狀態
     vTaskDelay(pdMS_TO_TICKS(300));        // 延遲 300ms (讓出 CPU)
   // osDelay(1);
 }
 /* USER CODE END _Led2Task */
}
/* USER CODE BEGIN Header__Led3Task */
/**
* @brief Function implementing the Led3Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header__Led3Task */
void _Led3Task(void *argument) // 任務函式 (Led3Task)
{
 /* USER CODE BEGIN _Led3Task */
 /* Infinite loop */
 for(;;) // 任務無窮迴圈
 {
     HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); // 切換 LED3 (GPIOB Pin 14) 狀態
     vTaskDelay(pdMS_TO_TICKS(1000));         // 延遲 1000ms (讓出 CPU)
   //osDelay(1);
 }
 /* USER CODE END _Led3Task */
}
/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) // 錯誤處理函式
{
 /* USER CODE BEGIN Error_Handler_Debug */
 /* User can add his own implementation to report the HAL error return state */
 __disable_irq(); // 關閉所有中斷，避免更多干擾
 while (1) // 進入死結迴圈，停止程式運行
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
void assert_failed(uint8_t *file, uint32_t line) // 斷言失敗處理函式 (除錯用)
{
 /* USER CODE BEGIN 6 */
 /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
 /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */