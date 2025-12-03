/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Ford EEC-IV Diagnostic Reader
 *
 * Description:
 * This firmware decodes diagnostic blink codes from Ford EEC-IV ECU.
 * The ECU outputs pulses (~1 second ON) with timing pauses indicating:
 *
 *   - Digits separated by ~4 seconds
 *   - Codes separated by ~6 seconds
 *
 * The firmware measures time between pulse edges via EXTI interrupt and sends
 * decoded events via UART in text format for PC / Diagnostic App.
 *
 * Hardware:
 *  - Input PB9
 *  - UART1 output to PC
 *
 * Protocol Output Examples:
 *   EV:READY
 *   EV:PULSE
 *   EV:DIGIT=2
 *   EV:CODE-END
 *   EV:SESSION-END
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
    WAITING_PULSES,
    WAITING_DIGIT_PAUSE,
    WAITING_CODE_PAUSE,
    SESSION_IDLE
} DecodeState;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PULSE_ON_TIME    500U
#define BETWEEN_PULSES   500U
#define DIGIT_PAUSE      3000U   // пауза между цифрами
#define CODE_PAUSE       4000U   // пауза между кодами
#define SESSION_PAUSE    10000U  // конец сессии
#define DEBOUNCE_MS      100U    // антидребезг фронтов (минимум)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint32_t lastPulseTime = 0;
volatile uint8_t pulseCount = 0;
volatile uint8_t digitIndex = 0;
volatile uint8_t digits[2] = {0,0};
volatile DecodeState state = SESSION_IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void send_json(const char* eventName, int value);
void generateTestCode(uint8_t tens, uint8_t ones);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  lastPulseTime = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* USER CODE BEGIN 3 */
	  uint32_t now = HAL_GetTick();
	  uint32_t silence = now - lastPulseTime;

	  if(state == WAITING_DIGIT_PAUSE && silence >= 3000 && pulseCount > 0) {
		  // завершилась цифра
		  send_json("digit", pulseCount);
		  digits[digitIndex++] = pulseCount;
		  pulseCount = 0;

		  state = WAITING_CODE_PAUSE;
	  }

	  if(state == WAITING_CODE_PAUSE && silence >= 4000 && digitIndex > 0) {
		  int code = digits[0] * 10 + digits[1];
		  send_json("code", code);

		  // reset
		  digitIndex = 0;
		  digits[0] = digits[1] = 0;
		  pulseCount = 0;

		  state = WAITING_PULSES;
	  }

	  // end of session if >10s паузы
	  if(state != SESSION_IDLE && silence >= 10000 && digitIndex == 0 && pulseCount == 0) {
		  send_json("session_end", -1);
		  state = SESSION_IDLE;
	  }

	  HAL_Delay(25);
	  /* USER CODE END 3 */
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint32_t now = HAL_GetTick();
	  uint32_t silence = now - lastPulseTime;

	  if(state == WAITING_DIGIT_PAUSE && silence >= 3000 && pulseCount > 0) {
		  // завершилась цифра
		  send_json("digit", pulseCount);
		  digits[digitIndex++] = pulseCount;
		  pulseCount = 0;

		  state = WAITING_CODE_PAUSE;
	  }

	  if(state == WAITING_CODE_PAUSE && silence >= 4000 && digitIndex > 0) {
		  int code = digits[0] * 10 + digits[1];
		  send_json("code", code);

		  // reset
		  digitIndex = 0;
		  digits[0] = digits[1] = 0;
		  pulseCount = 0;

		  state = WAITING_PULSES;
	  }

	  // end of session if >10s паузы
	  if(state != SESSION_IDLE && silence >= 10000 && digitIndex == 0 && pulseCount == 0) {
		  send_json("session_end", -1);
		  state = SESSION_IDLE;
	  }

	  HAL_Delay(25);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void send_json(const char* eventName, int value){
    char b[64];
    if(value >= 0)
        sprintf(b, "{\"event\":\"%s\",\"value\":%d,\"t\":%lu}\r\n", eventName, value, HAL_GetTick());
    else
        sprintf(b, "{\"event\":\"%s\",\"t\":%lu}\r\n", eventName, HAL_GetTick());

    HAL_UART_Transmit(&huart1,(uint8_t*)b,strlen(b),100);
}

/**
 * @brief EXTI interrupt handler for diagnostic pulse signal.
 * Called automatically on PB9 rising edge.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin != GPIO_PIN_9) return;

    uint32_t now = HAL_GetTick();
    uint32_t silence = now - lastPulseTime;
    lastPulseTime = now;

    // debounce (<100ms — шум)
    if(silence < 100) return;

    if(state == SESSION_IDLE) {
		state = WAITING_PULSES;
        digitIndex = 0;
        pulseCount = 0;
        digits[0] = digits[1] = 0;
    }

    pulseCount++;
    send_json("pulse", pulseCount);

    state = WAITING_DIGIT_PAUSE;
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
