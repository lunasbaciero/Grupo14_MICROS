/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "waveplayer.h"
#include "File_Handling.h"
#include "string.h"
#include <stdio.h>
#include "i2c_lcd.h"
#include <stdlib.h>
#include <stm32f4xx.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TAM_STRING 40

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

I2C_LCD_HandleTypeDef lcd1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern ApplicationTypeDef Appli_state;
extern AUDIO_PLAYBACK_StateTypeDef AudioState;
extern uint32_t uwVolume = 75;
extern FRESULT fresult;
extern FILELIST_FileTypeDef TxtList;
extern uint16_t NumObs;

int IsFinished = 0;
volatile uint32_t valpot;
volatile uint8_t idx = 0;

volatile estado = 0;
volatile uint8_t t_linea = 0;
volatile uint8_t t_letra_deseado = 0;

volatile uint8_t cambio_linea = 1;
volatile uint8_t primera_linea = 1;

FIL filetxt;
char buffertxt[4096];

// Banderas botones
volatile boton1 = 0;	// izquierda
volatile boton2 = 0;	// derecha
volatile boton0 = 0;	// centro

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t t_button = 0;

		if(HAL_GetTick() - t_button > 20){

			if (GPIO_Pin == GPIO_PIN_0)
			{
				boton0 = 1;
				//AudioState = AUDIO_STATE_NEXT;
				if(estado == 2){
					if (AudioState == AUDIO_STATE_PLAY)
					{
						AudioState = AUDIO_STATE_PAUSE;
						HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
						boton0=0;
					}

					if (AudioState == AUDIO_STATE_WAIT)
					{
						AudioState = AUDIO_STATE_RESUME;
						HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
						boton0=0;
					}
				}
			}

			else if(GPIO_Pin == GPIO_PIN_1)
			{
				boton1 = 1;


			}

			else if (GPIO_Pin == GPIO_PIN_2)
			{
				boton2 = 1;
			}

		t_button = HAL_GetTick();
		}
}

void LeerPotenciometro(void){
	// No es necesario comprobar quién ha generado la interrupción
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	valpot = HAL_ADC_GetValue(&hadc1);
	uwVolume = valpot*100/4095;
	if(uwVolume < 10)
		uwVolume = 10;
	else if(uwVolume > 90)
		uwVolume = 90;
	AUDIO_OUT_SetVolume(uwVolume);
}

void init_lcds(void){
	lcd1.hi2c = &hi2c2;
	lcd1.address = 0x4E;
	lcd_init(&lcd1);
}

void lcd_send_two_lines(const char *text) {

    char line1[17] = {0}; // Línea 1 (16 caracteres + terminador nulo)
    char line2[17] = {0}; // Línea 2 (16 caracteres + terminador nulo)
    int i=0;
    // Copiar los primeros 16 caracteres a la primera línea
    strncpy(line1, text, 16);

    // Si hay más de 16 caracteres, copiar los siguientes 16 a la segunda línea
    if (strlen(text) > 16) {
        strncpy(line2, text + 16, 16);
    }

    while (line2[i] != '\0') {  // Recorrer la cadena
            if (line2[i] == '.') {  // Si encontramos un punto
                line2[i] = '\0';  // Reemplazamos el punto por un carácter nulo
                break;  // Detenemos la búsqueda
            }
            i++;
        }
    while (line1[i] != '\0') {  // Recorrer la cadena
            if (line1[i] == '\r') {  // Si encontramos un punto
                line1[i] = ' ';  // Reemplazamos el punto por un carácter nulo
                break;  // Detenemos la búsqueda
            }
            i++;
         }

    while (line2[i] != '\0') {  // Recorrer la cadena
                if (line2[i] == '\r') {  // Si encontramos un punto
                    line2[i] = ' ';  // Reemplazamos el punto por un carácter nulo
                    break;  // Detenemos la búsqueda
                }
                i++;
    }

    // Mostrar las líneas en el LCD
    lcd_gotoxy(&lcd1, 0, 0);
    lcd_puts(&lcd1, line1);

    lcd_gotoxy(&lcd1, 0, 1);
    lcd_puts(&lcd1, line2);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	t_linea++;
}

void LeerFicheroTexto(char* buffer){
	UINT bytesRead;
	uint8_t i = 0;
	uint8_t buffer_correcto = 0;

	strcpy(buffer,"");
	fresult = f_read(&filetxt, buffer, 4096, &bytesRead);
	if(fresult == FR_OK){
		//Paramos al final de la última linea completa
		buffer_correcto = 0;
		for(i = bytesRead-1; i>0 && buffer_correcto == 0; i--){
			if(buffer[i]=='\n'){
				buffer[i+1] = '\0';
				fresult = f_lseek(&filetxt,f_tell(&filetxt) + i - (bytesRead));
				buffer_correcto = 1;
			}
		}
	}
}


void EnviarLetra(char* buffer_letra){
	static char linea[TAM_STRING] = "";
	char t_deseado_str[TAM_STRING];

	if(cambio_linea){
		if(primera_linea == 1){
			strncpy(linea,strtok(buffer_letra, "\n"),TAM_STRING);
			primera_linea = 0;
		}
		else {
			strncpy(linea,strtok(NULL, "\n"),TAM_STRING);
		}

		if(linea[0] == '\0'){
			strcpy(buffer_letra,"");
			LeerFicheroTexto(buffer_letra);
			strncpy(linea,strtok(buffer_letra, "\n"),TAM_STRING);
		}

		if(!strcmp("&",linea)){
			AudioState = AUDIO_STATE_STOP;
			uwVolume = 10;
			AUDIO_OUT_SetVolume(uwVolume);
			return;
		}

		//SACAR EL TIEMPO Y CONVERTIRLO
		strncpy(t_deseado_str,linea,2);
		t_letra_deseado = (t_deseado_str[0]-'0')*10+(t_deseado_str[1]-'0');

		strcpy(linea,linea+2);

		lcd_send_two_lines(linea);
		cambio_linea = 0;
	}
	else if(t_linea >= t_letra_deseado){
		t_linea = 0;
		cambio_linea = 1;
		lcd_gotoxy(&lcd1,0,0);
		lcd_clear(&lcd1);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  init_lcds();

  //HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    switch(estado)
    {
    case 0:
    	lcd_gotoxy(&lcd1,0,0);
    	lcd_puts(&lcd1, "CONECTE USB");
    	if (Appli_state == APPLICATION_READY)
    	{
    		estado = 1;
    		lcd_gotoxy(&lcd1,0,0);
        	lcd_clear(&lcd1);
        	Mount_USB();
        	AUDIO_StorageParse();
    	}
    	break;

    case 1:
    	lcd_send_two_lines((char *)TxtList.file[idx].name);
    	//Mount_USB();
    	if (IsFinished)
    		IsFinished = 0;

    	//estado = 2;
    	if (boton0)
    	{
    		estado = 2;
    		boton0 = 0;
    		lcd_gotoxy(&lcd1,0,0);
    		lcd_clear(&lcd1);
    		AudioState = AUDIO_STATE_RESUME;
    		primera_linea = 1;
    	}
    	if (boton1)
    	{
    		if(idx>0)
    		{
    			idx--;
    		}
    		else
    		{
    			idx = NumObs-1;
    		}
    		boton1 = 0;
    		lcd_gotoxy(&lcd1,0,0);
    		lcd_clear(&lcd1);
    	}

    	if (boton2)
    	{
    		if(idx<NumObs-1)
    		{
    			idx++;
    		}
    		else
    		{
    		    idx = 0;
    		}
    		boton2 = 0;
    		lcd_gotoxy(&lcd1,0,0);
    		lcd_clear(&lcd1);
    	}

    	break;

    case 2:
    	//lcd_gotoxy(&lcd1,0,0);
    	//lcd_puts(&lcd1, "ESTADO 2");

    	if(fresult == FR_OK) // USB montado correctamente
    	{
    		Mount_USB();
    		AUDIO_PLAYER_Start(idx);
    		HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
    		f_open(&filetxt, (char*)TxtList.file[idx].name, FA_READ);
    		LeerFicheroTexto(buffertxt);

    		if (IsFinished)
    			IsFinished = 0;

    		while (!IsFinished)
    		{
    			AUDIO_PLAYER_Process(TRUE);
    			EnviarLetra(buffertxt);
    		    LeerPotenciometro();
    		    if(boton1)
    		    {
    		    	boton1 = 0;
    		    	AudioState = AUDIO_STATE_STOP;
    		    	uwVolume = 10;
    		    	AUDIO_OUT_SetVolume(uwVolume);

    		    	// Silenciamos el audio porque queda un residuo de la canción al pararla


    		    }
    		    if (AudioState == AUDIO_STATE_STOP)
    		    {
    		    	IsFinished = 1;
    		    	estado = 1;
    	    		HAL_TIM_OC_Stop_IT(&htim3, TIM_CHANNEL_1);
    	    		f_lseek(&filetxt, 0);
    	    		f_close(&filetxt);
    	    		t_letra_deseado=0;
    	    		t_linea=0;
    	    		cambio_linea = 1;
    		    	AudioState = AUDIO_STATE_IDLE;
    		    }

    		}
    	}
    	else
    		estado = 0;

    	break;

    case 3:
    	break;
    }


    //if (Appli_state == APPLICATION_READY)
    //{
    //	Mount_USB();

    //}


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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  hi2c2.Init.ClockSpeed = 100000;
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_44K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 999;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
