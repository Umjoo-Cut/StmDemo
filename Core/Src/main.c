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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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

/* USER CODE BEGIN PV */
// =========================
// 모드 전환
// =========================
#define UART_ENABLED        0      // 0=데모, 1=실제 UART

// =========================
// 임계값 / 타이밍
// =========================
#define DELTA_THRESHOLD     200    // 음주 판정 차이값 (첫 측정 대비)
#define MQ3_SAMPLE_COUNT    5      // 측정 횟수
#define MQ3_SAMPLE_INTERVAL 1000   // 측정 간격 (ms)
#define MAX_RETRY           3      // 최대 재측정 횟수
#define DEMO_RESPONSE_TIME  2000   // 데모 모드 응답 시뮬 시간 (ms)
#define RESULT_DISPLAY_TIME 3000   // 결과 표시 시간 (ms)
#define PRESSURE_LOST_TIMEOUT 10000  // 압력 해제 타임아웃 (30초 : 테스트 10초)
#define FAIL_BLINK_TIME     5000  // FAIL 시 Engine LED 깜빡 시간 (10초)

// =========================
// 상태 머신
// =========================
typedef enum {
    STATE_IDLE,           // 대기
    STATE_ARMED,          // 시동 ON, RPi 준비 대기
    STATE_WAIT_DETECT,    // 운전자 감지 버튼 대기
    STATE_MEASURING,      // MQ-3 측정 중
    STATE_WAIT_RESULT,    // RPi 판정 대기
    STATE_PASS,           // 통과
    STATE_FAIL_DRUNK      // 음주 차단
} DMS_State_t;

DMS_State_t current_state = STATE_IDLE;
uint32_t state_enter_time = 0;
uint32_t mq3_value = 0;             // 최종 판정용 값 (최대 delta)
uint32_t mq3_baseline = 0;          // 첫 측정값 (기준)
uint8_t retry_count = 0;            // 재측정 카운터
uint8_t start_btn_prev = 1;
uint8_t detect_btn_prev = 1;

// 측정 중 압력 감시용
uint8_t is_drunk_detected = 0;      // 측정 중 음주 감지 플래그
uint32_t pressure_lost_start = 0;   // PASS 상태에서 압력 해제 시각 추적
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */
// =========================
// LED 제어
// =========================
void LED_AllOff(void) {
    HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
}

// =========================
// 엔진 LED 제어
// =========================
void Engine_ON(void) {
    HAL_GPIO_WritePin(Engine_LED_GPIO_Port, Engine_LED_Pin, GPIO_PIN_SET);
}

void Engine_OFF(void) {
    HAL_GPIO_WritePin(Engine_LED_GPIO_Port, Engine_LED_Pin, GPIO_PIN_RESET);
}

// =========================
// MQ-3 측정
// =========================
uint32_t Read_MQ3(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t value = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return value;
}

// =========================
// 압력(버튼) 상태 읽기
// Pull-up이라 누르면 0, 떼면 1
// 압력 유지 = 0 반환
// =========================
uint8_t Is_Pressure_Active(void) {
    return (HAL_GPIO_ReadPin(DETECT_BTN_GPIO_Port, DETECT_BTN_Pin) == 0);
}

// =========================
// MQ-3 측정 (압력 감시 + delta 판정)
// 반환값:
//   0 = 정상 (모든 delta < 200)
//   1 = 음주 감지 (delta >= 200 한 번이라도)
//   2 = 측정 중 압력 해제 (30초 이상)
// =========================
uint8_t Measure_MQ3_With_Pressure(void) {
    // 첫 측정 전에도 압력 확인
    if (!Is_Pressure_Active()) {
        return 2;
    }

    mq3_baseline = Read_MQ3();   // 첫 측정값 = 기준
    uint32_t max_delta = 0;
    uint8_t drunk_flag = 0;

    for (int i = 1; i < MQ3_SAMPLE_COUNT; i++) {
        // 1초 대기하면서 압력 감시
        uint32_t wait_start = HAL_GetTick();

        while (HAL_GetTick() - wait_start < MQ3_SAMPLE_INTERVAL) {
            // ⭐ 압력 해제되면 즉시 중단!
            if (!Is_Pressure_Active()) {
                mq3_value = max_delta;   // 지금까지 값 저장
                return 2;
            }
            HAL_Delay(10);
        }

        // 압력 유지된 상태로 측정
        uint32_t v = Read_MQ3();
        int32_t delta = (int32_t)v - (int32_t)mq3_baseline;
        if (delta < 0) delta = -delta;

        if ((uint32_t)delta > max_delta) {
            max_delta = (uint32_t)delta;
        }

        if ((uint32_t)delta >= DELTA_THRESHOLD) {
            drunk_flag = 1;
        }
    }

    mq3_value = max_delta;
    return drunk_flag ? 1 : 0;
}

// =========================
// 부저 패턴
// =========================
void Buzzer_Alert(void) {
    for (int i = 0; i < 5; i++) {
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
        HAL_Delay(200);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_Delay(100);
    }
}

// =========================
// UART 송신
// =========================
void UART_Send(const char* msg) {
#if UART_ENABLED
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%s\n", msg);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 100);
#else
    (void)msg;
#endif
}

// =========================
// 상태 전환
// =========================
void Change_State(DMS_State_t new_state) {
    current_state = new_state;
    state_enter_time = HAL_GetTick();
    LED_AllOff();
}

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  // 시작: IDLE 상태, 엔진 OFF (시동 차단)
  Change_State(STATE_IDLE);
  Engine_OFF();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    uint32_t now = HAL_GetTick();
	    uint32_t elapsed = now - state_enter_time;

	    // 버튼 입력 감지
	    uint8_t start_btn_now = HAL_GPIO_ReadPin(START_BTN_GPIO_Port, START_BTN_Pin);
	    uint8_t detect_btn_now = HAL_GPIO_ReadPin(DETECT_BTN_GPIO_Port, DETECT_BTN_Pin);

	    uint8_t start_btn_pressed = (start_btn_prev == 1 && start_btn_now == 0);
	    uint8_t detect_btn_pressed = (detect_btn_prev == 1 && detect_btn_now == 0);

	    start_btn_prev = start_btn_now;
	    detect_btn_prev = detect_btn_now;

	    switch (current_state)
	    {
	        case STATE_IDLE:
	            // 대기: 모든 LED OFF, 엔진 OFF
	            Engine_OFF();

	            if (start_btn_pressed) {
	                retry_count = 0;
	                UART_Send("REQ:START");
	                Change_State(STATE_ARMED);
	            }
	            break;

	        case STATE_ARMED:
	            HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_SET);

	            if (elapsed >= DEMO_RESPONSE_TIME) {
	                Change_State(STATE_WAIT_DETECT);
	            }
	            break;

	        case STATE_WAIT_DETECT:
	            // 운전자 감지 버튼 대기 (Yellow 깜빡)
	            if ((elapsed / 500) % 2 == 0) {
	                HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_SET);
	            } else {
	                HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_RESET);
	            }

	            if (detect_btn_pressed) {
	                UART_Send("DETECT:ON");
	                Change_State(STATE_MEASURING);
	            }
	            break;

	        case STATE_MEASURING:
	        {
	            // MQ-3 측정 (압력 감시 + delta 판정)
	            HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_SET);

	            uint8_t result = Measure_MQ3_With_Pressure();

	            // MQ3 값 전송 (디버그용)
	            char msg_buf[32];
	            snprintf(msg_buf, sizeof(msg_buf), "MQ3_DELTA:%lu", mq3_value);
	            UART_Send(msg_buf);

	            if (result == 2) {
	                // ⭐ 측정 중 압력 해제 → IDLE 복귀 (완전 초기화!)
	                UART_Send("PRESSURE_LOST_IN_MEASURING");
	                retry_count = 0;                        // 재시도 카운터도 리셋
	                is_drunk_detected = 0;                  // 음주 플래그 리셋
	                Change_State(STATE_IDLE);               // ⭐ IDLE 복귀
	            } else {
	                // 정상 측정 완료 → 결과 대기
	                is_drunk_detected = (result == 1) ? 1 : 0;
	                Change_State(STATE_WAIT_RESULT);
	            }
	            break;
	        }

	        case STATE_WAIT_RESULT:
	            // RPi 판정 대기 (Yellow 천천히 깜빡)
	            if ((elapsed / 300) % 2 == 0) {
	                HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
                    HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);
	            } else {        
	                HAL_GPIO_WritePin(Yellow_LED_GPIO_Port, Yellow_LED_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
                    HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_RESET);
	            }

	            if (elapsed >= DEMO_RESPONSE_TIME) {
	                if (is_drunk_detected) {
	                    Change_State(STATE_FAIL_DRUNK);
	                } else {
	                    Change_State(STATE_PASS);
	                }
	            }
	            break;

	        case STATE_PASS:
	            // 🟢 통과: Engine LED ON 유지
	            HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
	            Engine_ON();

	            // 운전 중 압력 감시 (운전자 이탈 감지)
	            if (!Is_Pressure_Active()) {
	                // 압력 해제됨
	                if (pressure_lost_start == 0) {
	                    pressure_lost_start = HAL_GetTick();   // 시각 기록
	                }

	                // 30초 이상 압력 없음 → 재시도
	                if (HAL_GetTick() - pressure_lost_start >= PRESSURE_LOST_TIMEOUT) {
	                    UART_Send("PRESSURE_LOST");
	                    pressure_lost_start = 0;
	                    Change_State(STATE_WAIT_DETECT);   // 재시도
	                }
	            } else {
	                // 압력 다시 잡힘 → 카운터 리셋
	                pressure_lost_start = 0;
	            }

	            // 수동 종료 (시동 끄기)
	            if (start_btn_pressed) {
	                pressure_lost_start = 0;
	                Change_State(STATE_IDLE);
	            }
	            break;

	        case STATE_FAIL_DRUNK:
	            // 🔴 음주 차단
	            HAL_GPIO_WritePin(Red_LED_GPIO_Port, Red_LED_Pin, GPIO_PIN_SET);

	            // 부저 1회만 (진입 직후)
	            if (elapsed < 50) {
	                Buzzer_Alert();
	            }

	            // Engine LED 5초간 깜빡 (0.2초 간격)
	            if (elapsed < FAIL_BLINK_TIME) {
	                if ((elapsed / 200) % 2 == 0) {
	                    Engine_ON();
	                } else {
	                    Engine_OFF();
	                }
	            } else {
	                Engine_OFF();   // 5초 후 OFF 고정
	            }

	            // 10초 후 재시도 판단
	            if (elapsed >= FAIL_BLINK_TIME) {
	                retry_count++;

	                if (retry_count < MAX_RETRY) {
	                    Change_State(STATE_WAIT_DETECT);
	                } else {
	                    Change_State(STATE_IDLE);
	                }
	            }
	            break;
	    }

	    HAL_Delay(10);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/* USER CODE BEGIN 4 */

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
