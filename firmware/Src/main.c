/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <servomotor.h>
#include <mpu6050.h>
#include <moving_average.h>
#include <float_to_string.h>
//#include <stm32f103x6.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEFAULT_SMOOTHING 20
#define DEFAULT_FACTOR 1.4
#define ROUND_ANGLE_PACK

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

SD_MPU6050 position_sensor;

float sensor_roll, sensor_pitch, sensor_factor;
float roll_compensation, pitch_compensation;
size_t smoothing;

servo_t servo_roll;
servo_t servo_pitch;

maf_t roll_filter;
maf_t pitch_filter;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* -- System -- */
void init_system()
{
    smoothing = DEFAULT_SMOOTHING;
    if(init_maf(&roll_filter, smoothing) == 0) error_alert("Cannot init MAF");
    if(init_maf(&pitch_filter, smoothing) == 0) error_alert("Cannot init MAF");

    init_position_sensor();
    init_servomotors();
    init_uart_messaging();
    init_servomotors_motion();
    init_mailbox();
}

void error_alert(char *msg)
{
    char str[100];
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
    HAL_NVIC_DisableIRQ(TIM3_IRQn);

    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_SET);
    
    sprintf(str, "{\"type\":\"error\",\"text\":\"%s\"}\n\r", msg);
    HAL_UART_Transmit(&huart1, str, strlen(str), 100);
    
    HAL_Delay(500);

    HAL_GPIO_WritePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin, GPIO_PIN_RESET);
    
    HAL_NVIC_SystemReset();
}

/* -- Servomotors -- */
void init_servomotors()
{
    servo_status_t r;
    
    r = servo_init(&servo_roll, &htim3, TIM_CHANNEL_1);
    if(r != SERVO_STATUS_OK) error_alert("Cannot init servo_roll");

    r = servo_init(&servo_pitch, &htim3, TIM_CHANNEL_2);
    if(r != SERVO_STATUS_OK) error_alert("Cannot init servo_pitch");

    servo_set_offset(&servo_roll, 90);
    servo_set_offset(&servo_pitch, 90);
}

void init_servomotors_motion()
{
    HAL_TIM_Base_Start_IT(&htim3);
}

void servo_update()
{
    servo_set_position(&servo_roll, roll_compensation);
    servo_set_position(&servo_pitch, pitch_compensation);
}

/* -- Sensors -- */
void init_position_sensor()
{
    sensor_roll = 0;
    sensor_pitch = 0;
    sensor_factor = DEFAULT_FACTOR;

    HAL_Delay(500);
    
    SD_MPU6050_Result r;
    r = SD_MPU6050_Init(&hi2c2, &position_sensor, SD_MPU6050_Device_0, SD_MPU6050_Accelerometer_2G, SD_MPU6050_Gyroscope_250s);
    
    if(r != SD_MPU6050_Result_Ok)
    {
        error_alert("Cannot init MPU6050");
    }
    
    sensor_calc_position(&sensor_pitch, &sensor_roll);
    HAL_TIM_Base_Start_IT(&htim4);
}

#define MPU6050_I2C_ADDR	0xD0
#define MPU6050_I_AM	0x68
#define MPU6050_WHO_AM_I 0x75

/**
 * @brief Test the connection with the MPU6050 sensor. For bad connection, reset the MCU.
 */
void test_sensor()
{
    uint8_t temp;
    uint8_t WHO_AM_I = (uint8_t)MPU6050_WHO_AM_I;
    SD_MPU6050_Result r;

    /* Check who am I */
	  //------------------
		/* Send address */
		if(HAL_I2C_Master_Transmit(&hi2c2, MPU6050_I2C_ADDR, &WHO_AM_I, 1, 1000) != HAL_OK)
		{
        error_alert("MPU6050 connection fail");
		}

		/* Receive multiple byte */
		if(HAL_I2C_Master_Receive(&hi2c2, MPU6050_I2C_ADDR, &temp, 1, 1000) != HAL_OK)
		{
        error_alert("MPU6050 connection fail");
		}

		/* Checking */
		if(temp != MPU6050_I_AM)
		{
				error_alert("MPU6050 connection fail");
		}
	  //------------------
}


/**
 * @brief Update position and compensation informations
 */
void update_position_sensor()
{
    HAL_GPIO_WritePin(UPDATE_LED_GPIO_Port, UPDATE_LED_Pin, GPIO_PIN_SET);
    
    test_sensor();
    SD_MPU6050_Result r;
    r = SD_MPU6050_ReadAccelerometer(&hi2c2, &position_sensor);
    if(r != SD_MPU6050_Result_Ok)
    {
        error_alert("Cannot read MPU6050 Read Acce");
    }

    sensor_calc_position(&sensor_pitch, &sensor_roll);
    roll_compensation = -maf_filter(&roll_filter, sensor_roll);
    pitch_compensation = -maf_filter(&pitch_filter, sensor_pitch);

    HAL_GPIO_WritePin(UPDATE_LED_GPIO_Port, UPDATE_LED_Pin, GPIO_PIN_RESET);
}


#define SQUARE(v) ((v)*(v))
void sensor_calc_position(float *pitch, float *roll)
{
    *pitch = 180 * atan (position_sensor.Accelerometer_X/sqrt(SQUARE(position_sensor.Accelerometer_Y) + SQUARE(position_sensor.Accelerometer_Z)))/M_PI;
    *roll = 180 * atan (position_sensor.Accelerometer_Y/sqrt(SQUARE(position_sensor.Accelerometer_X) + SQUARE(position_sensor.Accelerometer_Z)))/M_PI;
    *pitch *= sensor_factor;
    *roll *= sensor_factor;
}

/* --- Smoothing -- */


/**
 * @brief Reset MAF with a new buffer length
 */
void change_smoothing(size_t value)
{
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
    
    servo_set_position(&servo_roll, 0);
    servo_set_position(&servo_pitch, 0);

    smoothing = value;
    
    delete_maf(&roll_filter);
    delete_maf(&pitch_filter);

    init_maf(&roll_filter, smoothing);
    init_maf(&pitch_filter, smoothing);
    
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* -- UART messaging -- */


/**
 * @brief Send position & compensation infos to TTL
 */
void uart_message_update()
{
    send_angle_pack(SENSOR_ANGLE_PACK, sensor_pitch, sensor_roll);
    send_angle_pack(SERVO_ANGLE_PACK, pitch_compensation, roll_compensation);
}


/**
 * @brief Initialize messaging timer interruptions
 */
void init_uart_messaging()
{
    HAL_TIM_Base_Start_IT(&htim2);
}

void send_value_pack(value_pack_target_t target, float value)
{
    char value_pack_str[100];
    char value_str[10];
    float_to_string(value, value_str);

    if(target == SMOOTHING_VALUE_PACK)
    {
        sprintf(value_pack_str, "{\"type\":\"value\", \"target\",\"smoothing\", \"value\":%s}\n", value_str);
    }
    else //FACTOR_VALUE_PACK
    {
        sprintf(value_pack_str, "{\"type\":\"value\", \"target\",\"factor\", \"value\":%s}\n", value_str);
    }
}

void send_angle_pack(angle_pack_target_t target, float pitch, float roll)
{
    char angle_pack_str[100];

#ifdef ROUND_ANGLE_PACK
    if(target == SENSOR_ANGLE_PACK)
    {
        sprintf(angle_pack_str, "{\"type\":\"angle\",\"pitch\":%d,\"roll\":%d, \"target\":\"sensor\"}\n", (long)pitch, (long)roll);
    }
    else //SERVO_ANGLE_PACK
    {
        sprintf(angle_pack_str, "{\"type\":\"angle\",\"pitch\":%d,\"roll\":%d, \"target\":\"servo\"}\n", (long)pitch, (long)roll);
    }
    
#else
    char roll_str[20], pitch_str[20];

    float_to_string(roll, roll_str);
    float_to_string(pitch, pitch_str);
    
    if(target == SENSOR_ANGLE_PACK)
    {
        sprintf(angle_pack_str, "{\"type\":\"angle\",\"pitch\":%s,\"roll\":%s, \"target\":\"sensor\"}\n", pitch_str, roll_str);    
    }
    else if(target == OFFSET_ANGLE_PACK)
    {
        sprintf(angle_pack_str, "{\"type\":\"angle\",\"pitch\":%s,\"roll\":%s, \"target\":\"offset\"}\n", pitch_str, roll_str);
    }
    else //SERVO_ANGLE_PACK
    {
        sprintf(angle_pack_str, "{\"type\":\"angle\",\"pitch\":%s,\"roll\":%s, \"target\":\"servo\"}\n", pitch_str, roll_str);
    }
#endif

    HAL_UART_Transmit(&huart1, angle_pack_str, strlen(angle_pack_str), 100);
}

void send_echo_pack(char *msg)
{
    char echo_pack_str[100];
    sprintf(echo_pack_str, "{\"type\":\"echo\",\"text\":\"%s\"}\n", msg);
    HAL_UART_Transmit(&huart1, echo_pack_str, strlen(echo_pack_str), 100);
}

void send_message_pack(char *msg)
{
    char message_pack_str[100];
    sprintf(message_pack_str, "{\"type\":\"message\",\"text\":\"%s\"}\n", msg);
    HAL_UART_Transmit(&huart1, message_pack_str, strlen(message_pack_str), 100);
}

/* --- Mailbox --- */

char mailbox_buffer[50];

void init_mailbox()
{
}

/**
 * @brief UART RX IRQ callback
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  for(int i = 0; i < 50; i++)
  {
      if(mailbox_buffer[i] == '\n')
      { 
          mailbox_buffer[i] = '\0';
          //send_echo_pack(mailbox_buffer);
          mailbox_message_received(mailbox_buffer);
          return;
      }
  }

  mailbox_buffer[49] = '\0';
  mailbox_message_received(mailbox_buffer);
}

void mailbox_listen()
{
    HAL_UART_Receive_IT(&huart1, mailbox_buffer, 50);
}

void mailbox_message_received(char *message)
{
    if(strcmp(message, "pause") == 0)
    {
        HAL_NVIC_DisableIRQ(TIM3_IRQn);
        HAL_NVIC_DisableIRQ(TIM4_IRQn);
    }
    else if(strcmp(message, "resume") == 0)
    {
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
    else
    {
        #if 0 /* problematic code */
        char pack_type[100];
        char pack_param[100];
        int pack_value;
        int pack_exp;

        sscanf(message, "type %s param %s value %de%d", pack_type, pack_param, &pack_value, &pack_exp);
        float float_value = 0;
        float_value = (float)pack_value * pow(10, pack_exp);

        //send_message_pack(pack_type);
        //send_message_pack(pack_param);
        //send_value_pack(SMOOTHING_VALUE_PACK, float_value);        
        
        if(strcmp(pack_type, "get") == 0)
        {
            if(strcmp(pack_param, "sensor") == 0)
            {
                send_angle_pack(SENSOR_ANGLE_PACK, sensor_pitch, sensor_roll);
            }
            else if(strcmp(pack_param, "servo") == 0)
            {
                send_angle_pack(SERVO_ANGLE_PACK, pitch_compensation, roll_compensation);
            }
            else if(strcmp(pack_param, "factor") == 0)
            {
                send_value_pack(FACTOR_VALUE_PACK, sensor_factor);
            }
            else if(strcmp(pack_param, "smoothing") == 0)
            {
                send_value_pack(SMOOTHING_VALUE_PACK, (float)smoothing);
            }
            else if(strcmp(pack_param, "offset") == 0)
            {
                send_angle_pack(OFFSET_ANGLE_PACK, servo_pitch.offset, servo_roll.offset);
            }
            else
            {
                send_message_pack("undefined param");
            }
        }
        else if(strcmp(pack_type, "set") == 0)
        {
            if(strcmp(pack_param, "servo_pitch") == 0)
            {
                pitch_compensation = float_value;
                servo_set_position(&servo_pitch, pitch_compensation);
            }
            else if(strcmp(pack_param, "servo_roll") == 0)
            {
                roll_compensation = float_value;
                servo_set_position(&servo_roll, roll_compensation);
            }
            else if(strcmp(pack_param, "roll_offset") == 0)
            {
                servo_set_offset(&servo_roll, float_value);
            }
            else if(strcmp(pack_param, "pitch_offset") == 0)
            {
                servo_set_offset(&servo_pitch, float_value);
            }
            else if(strcmp(pack_param, "sensor_factor") == 0)
            {
                sensor_factor = float_value;
            }
            else if(strcmp(pack_param, "sensor_smoothing") == 0)
            {
                change_smoothing((int)float_value);
            }
            else
            {
                send_message_pack("undefined param");
            }
        }
        else
        {
            send_message_pack("undefined pack type");
        }
        #endif
    }
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
  MX_I2C2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  init_system();
  
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    mailbox_listen();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 60;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 8;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_MultiProcessor_Init(&huart1, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, UPDATE_LED_Pin|ERROR_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : UPDATE_LED_Pin ERROR_LED_Pin */
  GPIO_InitStruct.Pin = UPDATE_LED_Pin|ERROR_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
  error_alert("HAL Error_Handler call");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
