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
#include "stdio.h"
#include "mcp23017.h"
#include <SPM_Modbus.h>
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
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//デバイス定義
int16_t model_no = 50;
int16_t version = 1;
int16_t sub_version = 0;

//シリアルデータ
uint8_t recv_data;
uint8_t rx_buf[50];
int rx_len=0;

//CANデータ
uint32_t can_id;
uint32_t can_dlc;
uint8_t can_data[8];

//アプリケーション変数
int16_t dev_id=1;
int16_t io_mode=0; //0通常動作　1:セーフティ動作
//通常動作時の入出力
int16_t io_in=0;
int16_t io_out=0;
//セーフティモード時の入出力
int16_t io_in_fillter = 0xff;	//入力のフィルター ビットが1のもののみセーフティが有効
int16_t io_norm_in=0x00;		//正常時の入力状態。これと異なる場合に異常と判定
int16_t io_norm_out=0x00;		//正常時の出力
int16_t io_anom_out=0xff;		//異常時の出力
int16_t io_state=0;				//0正常　1:異常
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  //IOexpander intialize
  MCP23017_HandleTypeDef hmcp;
  mcp23017_init(&hmcp, &hi2c1, MCP23017_ADDRESS_20);
  mcp23017_iodir(&hmcp, MCP23017_PORTA, MCP23017_IODIR_ALL_INPUT);
  mcp23017_iodir(&hmcp, MCP23017_PORTB, MCP23017_IODIR_ALL_OUTPUT);

  //CAN initialize
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  CAN_FilterTypeDef filter;
  filter.FilterIdHigh         = 0;                        //
  filter.FilterIdLow          = 0;                        //
  filter.FilterMaskIdHigh     = 0;                        //
  filter.FilterMaskIdLow      = 0;                        //
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;    //
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         //
  filter.FilterBank           = 0;                        //
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;    //
  filter.SlaveStartFilterBank = 14;                       //
  filter.FilterActivation     = ENABLE;                   //
  HAL_CAN_ConfigFilter(&hcan, &filter);

  //各レジスタ初期化
  SPM_ModbusInit();

  //レジスタ登録
  p_input_regs[0] = &model_no;   //モデルナンバー
  p_input_regs[1] = &version; //バージョン
  p_input_regs[2] = &sub_version; //サブバージョン
  p_input_regs[3] = &io_in; //サブバージョン
  p_input_regs[4] = &io_out; //サブバージョン
  p_input_regs[5] = &io_mode;
  p_input_regs[6] = &io_in_fillter;
  p_input_regs[7] = &io_norm_in;
  p_input_regs[8] = &io_norm_out;
  p_input_regs[9] = &io_anom_out;
  p_input_regs[10] = &io_state;

  p_holding_regs[0] = &dev_id;
  p_holding_regs[1] = &io_in;
  p_holding_regs[2] = &io_out;
  p_holding_regs[3] = &io_mode;
  p_holding_regs[4] = &io_in_fillter;
  p_holding_regs[5] = &io_norm_in;
  p_holding_regs[6] = &io_norm_out;
  p_holding_regs[7] = &io_anom_out;
  p_holding_regs[8] = &io_state;

  //Cyclic Function登録
  cycfunc0.rx_len = 2;
  cycfunc0.rx_adr[0] = &io_in;
  cycfunc0.rx_adr[1] = &io_out;
  cycfunc0.tx_len = 1;
  cycfunc0.tx_adr[0] = &io_out;

  ParamLoad();
  io_out=0;
  io_state=0;
  SPM_ModbusSetAddress(dev_id);

  //uart2 interrupt
  HAL_UART_Receive_IT(&huart1, &recv_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//IO動作
	  	mcp23017_read_gpio(&hmcp, MCP23017_PORTA);	//入力を読み取り
  	    io_in = hmcp.gpio[MCP23017_PORTA];	//入力を取得
	    if(io_mode == 0){
	    	//通常動作時
	    	hmcp.gpio[MCP23017_PORTB] = (uint8_t)io_out;	//書き込み
	    }else{
	    	//セーフティモード時
	    	uint8_t io_in_fix = io_in & (uint8_t)io_in_fillter;	//入力のフィルタ
	    	if(io_in_fix != io_norm_in) io_state = 1;	//入力を判定
	    	if( io_state == 0){
	    		//正常時の出力
	    		io_out = io_norm_out;
	    	}else{
	    		//異常時の出力
	    		io_out = io_anom_out;
	    	}
	    	hmcp.gpio[MCP23017_PORTB] = (uint8_t)io_out;	//書き込み
	    }
	    mcp23017_write_gpio(&hmcp, MCP23017_PORTB);	//出力の書き込み

		  if(rx_len>0){
			  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
			  HAL_Delay(10);
			  uint8_t tx_buf[50];
			  int tx_len = SPM_ModbusTask(rx_buf, rx_len, tx_buf);
			  HAL_UART_Transmit(&huart1, tx_buf, tx_len, 100);
			  rx_len=0;
			  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
		  }

		/*if(timer_1s==0){
			CAN_TxHeaderTypeDef TxHeader;
			uint32_t TxMailbox;
			uint8_t TxData[8];
			if(0 < HAL_CAN_GetTxMailboxesFreeLevel(&hcan)){
				  TxHeader.StdId = 0x555;                 // CAN ID
				  TxHeader.RTR = CAN_RTR_DATA;            //
				  TxHeader.IDE = CAN_ID_STD;              //
				  TxHeader.DLC = 8;                       //
				  TxHeader.TransmitGlobalTime = DISABLE;  //
				  TxData[0] = 'H';
				  TxData[1] = 'E';
				  TxData[2] = 'L';
				  TxData[3] = 'L';
				  TxData[4] = 'O';
				  TxData[5] = '!';
				  TxData[6] = '!';
				  TxData[7] = '!';
				  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
			}

		    int8_t buf[50];
		    int16_t l = sprintf(buf,"PA=%d, PB=%d\r\n",
		    				hmcp.gpio[MCP23017_PORTA],
							hmcp.gpio[MCP23017_PORTB]);
		    HAL_UART_Transmit( &huart1, buf, l, 0xFFFF);
		}*/
		HAL_Delay(1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.Timing = 0x00201D2B;
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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
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
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        can_id = (RxHeader.IDE == CAN_ID_STD)? RxHeader.StdId : RxHeader.ExtId;     // ID
        can_dlc = RxHeader.DLC;                                                     // DLC
        can_data[0] = RxData[0];                                                    // Data
        can_data[1] = RxData[1];
        can_data[2] = RxData[2];
        can_data[3] = RxData[3];
        can_data[4] = RxData[4];
        can_data[5] = RxData[5];
        can_data[6] = RxData[6];
        can_data[7] = RxData[7];
    }
    int8_t buf[50];
    int16_t l = sprintf(buf,"ID=%d, %s\r\n",can_id, can_data);
    HAL_UART_Transmit( &huart1, buf, l, 0xFFFF);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART1)
   {
	   rx_buf[rx_len] = recv_data;
	   rx_len++;
       HAL_UART_Receive_IT(&huart1, &recv_data, 1);
   }
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
