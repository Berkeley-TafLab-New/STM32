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
#include "stdint.h"
#include "wit_c_sdk.h"//wit sdk
#include "ring_buffer.h"
#include "AS5600.h"
#include "servo_controls.h"
#include "rudder_control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COMMAND_MAX_LENGTH 64 //Uart Commands max size
#define RESPONSE_MAX_LENGTH 128// Uart response max size

//wit defines ->
#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define GPS_UPDATE    0x10 
#define READ_UPDATE		0x80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
//wit variables ->
static char s_cDataUpdate = 0;//wit update var
//defs
ring_buffer uart_ring_buffer;
ring_buffer uart2_ring_buffer;   // Separate buffer for USART2 (XBee)
uint8_t rx_data_uart2;           // 1-byte RX variable for USART2
uint8_t rx_data_s; // Single byte for receiving data
uint8_t rx_data_xbee; //single Byte from Xbee
uint8_t ucRxData = 0;//Single Byte Rx fOr Witmotion
char command_buffer[COMMAND_MAX_LENGTH]; // To hold the extracted command
char command_buffer_xbee[COMMAND_MAX_LENGTH];
uint32_t uiBuad = 115200;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

//wit function prototypes ->

static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
     //HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 1); //enable to get debug over STLINK

    return ch;
}
//uart interrupt ring buffer init
void System_Init(void) {
  // Initialize ring buffer
  ring_buffer_init(&uart_ring_buffer);
  ring_buffer_init(&uart2_ring_buffer);

  // Start UART reception in interrupt mode
  HAL_UART_Receive_IT(&huart3, &rx_data_s, 1); // initialising Stlink interrupts
  HAL_UART_Receive_IT(&huart2, &rx_data_uart2, 1); // initialising XBee interrupts
  HAL_UART_Receive_IT(&huart1, &ucRxData, 1);

  //Start the rudder
  // Initialize rudder control
  rudder_init(&htim1, TIM_CHANNEL_2);

  // Set initial rudder position
  rudder_target_angle = rudder_straight;
  rudder_current_angle = rudder_straight;

 }
 

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {


  if(huart->Instance==USART1)
	   {
	       WitSerialDataIn(ucRxData);
	       UART_Start_Receive_IT(huart, &ucRxData, 1);
	   }


  if (huart->Instance == USART3) { // Ensure this is for the correct UART instance
      // Add received byte to the ring buffer
      ring_buffer_put(&uart_ring_buffer, rx_data_s);

      // Check if we received a carriage return '\r' (end of command)
      if (rx_data_s == '\r') {
          uint8_t data;
          uint16_t index = 0;
          // Extract the command from the ring buffer
          while (ring_buffer_get(&uart_ring_buffer, &data) && data != '\r' && index < COMMAND_MAX_LENGTH - 1) {
              command_buffer[index++] = (char)data;
          }
          command_buffer[index] = '\0'; // Null-terminate the string
          // Process the command
          const char *response;
          if (strcmp(command_buffer, "hello") == 0) {
              response = "Hello to you too!\n";
          } else if (strcmp(command_buffer, "setzerouart") == 0) {
              if (AS5600_config_ZPOS(&hi2c1) == HAL_OK) {
                  response = "ZPOS set successfully.\n";
              } else {
                  response = "Failed to set ZPOS.\n";
              }
          } else {
              response = "Uh oh, something didn't work...\n";
          }
          // Transmit the response
          HAL_UART_Transmit(&huart3, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
          // Clear the command buffer for reuse
          memset(command_buffer, 0, COMMAND_MAX_LENGTH);
      }
      // Re-enable UART interrupt for next byte reception
      HAL_UART_Receive_IT(&huart3, &rx_data_s, 1);
  }
    
  if (huart->Instance == USART2) {
    ring_buffer_put(&uart2_ring_buffer, rx_data_uart2);




    //adding manual rudder controls

    if (rx_data_uart2 == '[') {
            // Move rudder left
            rudder_target_angle -= 5.0f;
            if (rudder_target_angle < (rudder_straight - rudder_range))
                rudder_target_angle = rudder_straight - rudder_range;

            char response[40];
            sprintf(response, "Rudder LEFT: %.1f degrees\r\n", rudder_target_angle);
            HAL_UART_Transmit(&huart2, (uint8_t *)response, strlen(response), 100);

        }
        else if (rx_data_uart2 == ']') {
            // Move rudder right
            rudder_target_angle += 5.0f;
            if (rudder_target_angle > (rudder_straight + rudder_range))
                rudder_target_angle = rudder_straight + rudder_range;

            char response[40];
            sprintf(response, "Rudder RIGHT: %.1f degrees\r\n", rudder_target_angle);
            HAL_UART_Transmit(&huart2, (uint8_t *)response, strlen(response), 100);
        }

    if (rx_data_uart2 == '\r') {
        uint8_t data;
        uint16_t index = 0;

        while (ring_buffer_get(&uart2_ring_buffer, &data) && data != '\r' && index < COMMAND_MAX_LENGTH - 1) {
            command_buffer_xbee[index++] = (char)data;
        }
        command_buffer_xbee[index] = '\0';

        const char *response;
        if (strcmp(command_buffer_xbee, "hello") == 0) {
            response = "Hello to you Xbee!\n";
        } else if (strcmp(command_buffer_xbee, "setzero") == 0) { // make it a switch case
            if (AS5600_config_ZPOS(&hi2c1) == HAL_OK) {
                response = "ZPOS set successfully via xbee.\n";
            } else {
                response = "Failed to set ZPOS via Xbee.\n";
            }
        } else {
            response = "Unrecognized command from XBee\n";
        }

        HAL_UART_Transmit(&huart2, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
        printf(response);
        memset(command_buffer_xbee, 0, COMMAND_MAX_LENGTH);
    }


    //HAL_UART_Receive_IT(&huart2, &rx_data_uart2, 1); // Restart interrupt
    UART_Start_Receive_IT(huart, &rx_data_uart2, 1);

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
  float fAcc[3], fGyro[3], fAngle[3], fYaw;
  int i;
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(CopeSensorData);

  System_Init();
  AutoScanSensor();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//sail
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//propellor
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);//rudder


  float angle;
  char str[] = "System Booted";
  HAL_UART_Transmit(&huart3, (uint8_t*)str, strlen(str), 2000);
  HAL_Delay(500);
  ServoController sail_servo;
  sail_servo.htim= &htim1;
  sail_servo.channel = TIM_CHANNEL_1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_StatusTypeDef i2c_status = AS5600_read_angle(&hi2c1, &angle);
	  if (i2c_status== HAL_OK){
		 // printf("the angle is %f \n", angle);
	  }
	  
	  if (i2c_status != HAL_OK) {
	      printf("Error reading angle from AS5600\n");
	      continue; // Skip to the next iteration
	  }
    
	  copy_wind_pos(&sail_servo, angle);
	  rudder_move_to();

    if(s_cDataUpdate)
    		{
    			printf("3");
            // Assumes AX, AY, AZ are sequential registers starting at AX
            // Assumes GX, GY, GZ are sequential registers starting at GX
            // Assumes Roll, Pitch, Yaw are sequential registers starting at Roll
    			for(i = 0; i < 3; i++)
    			{
    				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
    				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
    				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
    			}

    			// Combine high/low registers for 32-bit values and apply scaling
				// Use int32_t for intermediate signed 32-bit values
				// Use uint16_t cast for low words when combining to avoid sign extension issues
				// Use float for final calculated values

				// Longitude & Latitude Calculation
				int32_t iLon = ((int32_t)(short)sReg[LonH] << 16) | (uint16_t)sReg[LonL];
				int32_t iLat = ((int32_t)(short)sReg[LatH] << 16) | (uint16_t)sReg[LatL];

				// Convert from ddmm.mmmmm format (scaled by 100000) to decimal degrees
				float fLon_deg = (float)(iLon / 10000000); // Extract degrees (dd)
				float fLon_min = (float)((iLon % 10000000) / 100000.0f); // Extract minutes (mm.mmmmm)
				float fLongitude = fLon_deg + fLon_min / 60.0f;

				float fLat_deg = (float)(iLat / 10000000); // Extract degrees (dd)
				float fLat_min = (float)((iLat % 10000000) / 100000.0f); // Extract minutes (mm.mmmmm)
				float fLatitude = fLat_deg + fLat_min / 60.0f;

				// GPS Altitude (m)
				float fGpsAltitude = (float)(short)sReg[GPSHeight] / 10.0f;

				// GPS Heading/Course (°). Note: This is course over ground, not magnetic heading.
				float fGpsCourse = (float)(short)sReg[GPSYAW] / 100.0f;

				// GPS Ground Speed (km/h)
				int32_t iGpsSpeed = ((int32_t)(short)sReg[GPSVH] << 16) | (uint16_t)sReg[GPSVL];
				float fGpsSpeed_kmh = (float)iGpsSpeed / 1000.0f;

				// Satellite Info & Accuracy Metrics
				int iSatellites = (uint16_t)sReg[SVNUM]; // Number of satellites is likely unsigned
				float fPDOP = (float)(short)sReg[PDOP] / 100.0f;
				float fHDOP = (float)(short)sReg[HDOP] / 100.0f;
				float fVDOP = (float)(short)sReg[VDOP] / 100.0f;


    			if(s_cDataUpdate & ACC_UPDATE)
    			{
    				printf("acc:%.3f %.3f %.3f\r\n", fAcc[0], fAcc[1], fAcc[2]);
    				s_cDataUpdate &= ~ACC_UPDATE;
    			}
    			if(s_cDataUpdate & GYRO_UPDATE)
    			{
    				printf("gyro:%.3f %.3f %.3f\r\n", fGyro[0], fGyro[1], fGyro[2]);
    				s_cDataUpdate &= ~GYRO_UPDATE;
    			}
    			if(s_cDataUpdate & ANGLE_UPDATE)
    			{
                    fYaw = (float)((unsigned short)sReg[Yaw]) / 32768 * 180.0;
    				printf("angle:%.3f %.3f %.3f(%.3f)\r\n", fAngle[0], fAngle[1], fAngle[2], fYaw);
    				s_cDataUpdate &= ~ANGLE_UPDATE;
    			}
    			if(s_cDataUpdate & MAG_UPDATE)
    			{
    				printf("mag:%d %d %d\r\n", sReg[HX], sReg[HY], sReg[HZ]);
    				s_cDataUpdate &= ~MAG_UPDATE;
    			}
				if(s_cDataUpdate & GPS_UPDATE)
				  {
								// Print the GPS data
					printf("GPS Lat: %.6f, Lon: %.6f, Alt: %.1fm\r\n", fLatitude, fLongitude, fGpsAltitude);
					printf("GPS Spd: %.3fkm/h, Course: %.2fdeg\r\n", fGpsSpeed_kmh, fGpsCourse);
					printf("GPS Sats: %d, PDOP: %.2f, HDOP: %.2f, VDOP: %.2f\r\n", iSatellites, fPDOP, fHDOP, fVDOP);

					// Clear the GPS update flag
					//s_cDataUpdate &= ~GPS_UPDATE;
				}

                s_cDataUpdate = 0;
    		}
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  	  //UART_Start_Receive_IT(&huart2, &ucRxData, 1);
      //HAL_UART_Receive_IT(&huart1, &ucRxData, 1);

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PG7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
   /* Configure USART3 TX (PD8) and RX (PD9) */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3; // AF7 for USART3
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Configure USART2 TX (PD5) and RX (PD6) */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2; // AF7 for USART2
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  /* Configure USART1 TX (PA9) and RX (PA10) */
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL; // Or GPIO_PULLUP for RX if needed
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1; // AF7 for USART1
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_USART1 */


  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  HAL_UART_Transmit(&huart3, p_data, uiSize, uiSize*4);
}
static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;

            case LonL:
            case LonH:
            case LatL:
            case LatH:
            case GPSHeight:
            case GPSYAW:
            case GPSVL:
            case GPSVH:
            case SVNUM:
            case PDOP:
            case HDOP:
            case VDOP:
        s_cDataUpdate |= GPS_UPDATE;
            break;

            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	const uint32_t c_uiBaud[9] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
	int i, iRetry;
	
	for(i = 0; i < 9; i++)
	{
        uiBuad = c_uiBaud[i]; // literal waste of time for loop only kept it around cause it looks like a nice blocking loading sequence in serial
        HAL_Delay(250); // Settling time
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			HAL_Delay(200);
			if(s_cDataUpdate != 0)
			{
				printf("%lu baud find sensor\r\n\r\n", c_uiBaud[i]);
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
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
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
