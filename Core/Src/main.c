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
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "i2c_lcd.h" //Librería de https://github.com/alixahedi/i2c-lcd-stm32?tab=readme-ov-file#usage
#include <stm32f4xx_hal_i2c.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Lista del catálogo
typedef struct catalogo{
	char nombre[TAM_STRING];
	char nombre_mp4[TAM_STRING]; //nombre del archivo con extension .mp4 y path de la carpeta con el audio
	char nombre_txt[TAM_STRING]; //nombre del archivo con extension .txt y path de la carpeta con la letra
	struct catalogo* siguiente;
	struct catalogo* anterior;
} catalogo;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TAM_BUFFER_TEXT 1024
#define TAM_BUFFER_AUDIO 1024
#define TAM_STRING 100

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */


I2C_HandleTypeDef hi2c1;
I2C_LCD_HandleTypeDef lcd1;

//Lectura de USB
FATFS FatFs;      // Instancia de FatFS

FIL texto;          // Objeto de archivo para texto
FIL auidio;			// Objeto de archivo para audio

FRESULT fd_USB;     // Código de resultado de FatFS
UINT bytesRead;		//Bytes leidos

//Tiempo
uint8_t t_letra_deseado; //tiempo que debe permanecer la letra (s)
uint8_t t_letra_actual; //tiempo que lleva la letra en pantalla (s)
volatile uint8_t nueva_cancion; //Flag para indicar cambios de canción
char buffer_texto[TAM_BUFFER_TEXT];
catalogo *cat = NULL;

// banderas
volatile uint8_t FLAG_SIGUIENTE = 0;
volatile uint8_t FLAG_ANTERIOR = 0;
volatile int button_int=0;
volatile estado = 0;
volatile uint8_t valpot;
volatile uint8_t ma_tranfer_complete_flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

//USB y ficheros
uint8_t MontarUSB(void);
uint8_t AbrirFichero(char* nombre_fichero, FIL* file);
uint8_t LeerFicheroTexto(FIL* file, char* buffer); //No es responsable del envio por pantalla LCD
uint8_t ReproducirFicheroSonido(void); //También lo envía por DMA a la salida de audio.

//Procesado de información de ficheros de texto
catalogo* GenerarCatalogo(char* buffer_catalogo);
void EliminarCatalogo(catalogo** cat);
catalogo* MostrarCatalogo(catalogo* cat);
void EnviarLetra(char* buffer_letra);

// Botones y potenciómetro
void Button1Pressed(void);
void Button2Pressed(void);
void Button3Pressed(void);
void LeerPotenciometro(void);
void setvolumen(uint8_t volume);
int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



uint8_t MontarUSB(void){
	fd_USB = f_mount(&FatFs, "", 0);  // Monta el dispositivo USB

	if(fd_USB != FR_OK){
		//Eviar mensaje por LCD avisando de que no se encuentra un LCD

		lcd_gotoxy(&lcd1, 0, 1);
		lcd_puts(&lcd1, "ERROR AL CONECTAR USB");

		return 1; //Se considera 1 como error al montar el USB (Puede ser interesante cambiarlo por un enum)
	}
	return 0;
}

uint8_t AbrirFichero(char* nombre_fichero, FIL* file){

	fd_USB = f_open(file, nombre_fichero, FA_READ);

	if(fd_USB == FR_OK){
		return 0;
	}
	//Mensaje de error al abrir archivo

	lcd_gotoxy(&lcd1, 0, 1);
	lcd_puts(&lcd1, "ERROR AL ABRIR ARCHIVO");

	return 2; //Se considera 2 como error al abrir archivo (Puede ser interesante cambiarlo por un enum)
}

uint8_t LeerFicheroTexto(FIL* file, char* buffer){

	fd_USB = f_read(&file, buffer, sizeof(buffer) - 1, &bytesRead);
	if(fd_USB == FR_OK){
		buffer[bytesRead] = '\0';
		return 0;
	}
	lcd_gotoxy(&lcd1, 0, 1);
	lcd_puts(&lcd1, "ERROR AL LEER ARCHIVO");

	return 3; //Se considera 3 como error al leer el archivo (Puede ser interesante cambiarlo por un enum)

}

uint8_t ReproducirFicheroSonido(){
	static unsigned int bytesAudio = 0;
	static uint16_t buffer_audio[TAM_BUFFER_AUDIO];

	if(dma_transfer_complete_flag){
		fd_USB = f_read(&auidio, buffer_audio, TAM_BUFFER_AUDIO, &bytesAudio);

		if(fd_USB == FR_OK){
			//Si se ha acabado salimos de la función
			if(bytesAudio == 0){
				return 0;
			}
			//Si no, enviamos por dma
			HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, buffer_audio, TAM_BUFFER_AUDIO, DAC_ALIGN_12B_R);
		}

		//MENSAJE ERROR
		lcd_gotoxy(&lcd1, 0, 1);
		lcd_puts(&lcd1, "ERROR AL LEER ARCHIVO");

		return 3;
	}
	return 0;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypedef *hdac){
	dma_transfer_complete_flag = 1; // Actualizar flag
}


catalogo* GenerarCatalogo(char* buffer_catalogo){
	catalogo* cat, *cat_anterior;
	catalogo* primer_cat;

	char n[TAM_STRING];

	//Creamos la lista

	strncpy(n, strtok(texto_copia, "\n"), TAM_STRING);
	while (n != NULL) {

		if(cat == NULL){
			cat = (catologo*)malloc(sizeof(catalogo));
			
			strncpy(cat->nombre,"");
			strncpy(cat->nombre_mp4,"./audio/");
			strncpy(cat->nombre_txt,"./letra/");
			cat->siguiente = NULL;
			cat->anterior = NULL;
			
			primer_cat = cat;
		}
		else{
			cat_anterior = cat;

			cat->siguiente = malloc(sizeof(catalogo));
			cat = cat->siguiente;

			strncpy(cat->nombre,"");
			strncpy(cat->nombre_mp4,"./audio/");
			strncpy(cat->nombre_txt,"./letra/");
			cat->siguiente = NULL;
			cat->anterior = cat_anterior;
		}

		//Asignacion en la lista circular
		strncpy(cat->nombre, n);

		strncpy(cat->nombre_mp4, strcat(cat->nombre_mp4, n), TAM_STRING);
		strncpy(cat->nombre_mp4, strcat(cat->nombre_mp4,".mp4"), TAM_STRING);

		strncpy(cat->nombre_txt, strcat(cat->nombre_txt,n), TAM_STRING);
		strncpy(cat->nombre_txt, strcat(cat->nombre_txt,".txt"), TAM_STRING);

	    // Obtener la siguiente línea
	    strncpy(n, strtok(NULL, "\n"), TAM_STRING);
	}

	cat->siguiente = primer_cat;
	primer_cat->anterior = cat;

	return primer_cat;
}

void EliminarCatalogo(catalogo** cat){
	catalogo* siguiente, *actual;

	//Deshacemos primero la estructura circular
	actual = *cat;
	siguiente = actual->anterior;

	siguiente->siguiente = NULL;
	actual->anterior = NULL;

	while(actual != NULL){
		siguiente = actual->siguiente;
		free(actual);
		actual = siguiente;
	}
	*cat = NULL;
}

catalogo* MostrarCatalogo(catalogo* cat){

	catalogo* primer = cat;

	if(FLAG_SIGUIENTE){
		cat = cat->siguiente;
		FLAG_SIGUIENTE = 0;
	}

	if(FLAG_ANTERIOR){
		cat = cat->anterior;
		FLAG_ANTERIOR = 0;
	}

	if(cat == NULL){
		cat = primer;
	}

	lcd_gotoxy(&lcd1, 0, 1);
	lcd_puts(&lcd1, cat->nombre);

	return cat;
}

void EnviarLetra(char* buffer_letra){

	static char linea[TAM_STRING] = "";
	char t_deseado_str[TAM_STRING];

	if(t_letra_actual == 0){
		if(linea == "" || nueva_cancion == 1){
			strncpy(linea, strtok(buffer_letra, "\n"), TAM_STRING);
			nueva_cancion = 0;
		}
		else {
			strncpy(linea, strtok(buffer_letra, NULL), TAM_STRING);
		}
		//SACAR EL TIEMPO Y CONVERTIRLO
			strncpy(t_deseado_str,linea,2);
			t_letra_deseado = (t_deseado_str[0]-'0')*10+(t_deseado_str[1]-'0');

		lcd_gotoxy(&lcd1, 0, 1);
		lcd_puts(&lcd1, linea);
	}
	else if(t_letra_actual >= t_letra_deseado){
		t_letra_actual = 0;
	}
}

//Usar HAL_TIM_BASE_START para encenderlo al entrar en el estado ded la canción
void TIM6_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim6);  // Llamada al manejador de interrupción

    t_letra_actual += 1;

}


	// Definición de funciones declaradas
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	  if (GPIO_Pin == GPIO_PIN_0){ 		// Botón 1: Configurado en PA0
		  Button1Pressed();
	  }
	  else if (GPIO_Pin == GPIO_PIN_1){	// Botón 2: Configurado en PA1
		  Button2Pressed();
	  }
	  else if (GPIO_Pin == GPIO_PIN_2){	// Botón 3: Configurado en PA2
		  Button3Pressed();
	  }
}

	// Manejo del botón 1
	void Button1Pressed(){
	  switch(estado){
		case 1:	// Botón 1 en estado 1: Subir en la lista
		  	FLAG_ANTERIOR = 1;
		  	break;
		default:
			break;
	  }
	}

	// Manejo del botón 2
	void Button2Pressed(){
	  switch(estado){
		  case 1: // Botón 2 en estado 1: Seleccionar canción
		  	nueva_canción = 1;
		  	estado = 2;
		  	break;
		  case 2: // Botón 2 en estado 2: Pausar canción
			  
		  	break;
		  case 3: // Botón 2 en estado 3: Reanudar canción (resume)
			  
		  	break;
		  default:
		  	break;
	  }
}

	// Manejo del botón 3
	void Button3Pressed(){
	  switch(estado){
		  case 1: // Botón 3 en estado 1: Bajar en la lista
			  FLAG_SIGUIENTE;
			  break;
		  case 2: // Botón 3 en estado 2: Regreso a selección de canción
			  estado = 1;
			  f_close(&texto);
			  f_close(&auidio);
			  break;
		  default:
			  break;
	  }
}


	  // Lectura de potenciómetro (volumen)
	  void LeerPotenciometro(void){
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 1000); // Espera
		  valpot = HAL_ADC_GetValue(&hadc1);
	  }

	  // Determiar volumen según la lectura del potenciómetro
	  void setvolumen(uint8_t volume){
		  float volumen = (float)valpot/4095.0;

		  uint32_t salida_volumen = (uint32_t)(volumen * 4095);
		  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, salida_volumen);
		  HAL_Delay(10);
	  }


	int debouncer(volatile int* button_int, GPIO_TypeDef* GPIO_port, uint16_t GPIO_number){
		static uint8_t button_count=0;
		static int counter=0;

		if (*button_int==1){
			if (button_count==0) {
				counter=HAL_GetTick();
				button_count++;
			}
			if (HAL_GetTick()-counter>=20){
				counter=HAL_GetTick();
				if (HAL_GPIO_ReadPin(GPIO_port, GPIO_number)!=1){
					button_count=1;
				}
				else{
					button_count++;
				}
				if (button_count==4){ //Periodo antirebotes
					button_count=0;
					*button_int=0;
					return 1;
				}
			}
		}
		return 0;
	}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
uint8_t error;
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
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_FATFS_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */


    switch(estado){
           case 0: //Main menu
               //printf("Main menu\n");
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

               	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		   
		   if(MontarUSB() == 0){
		   	estado = 1;
			if(AbrirFicheroTexto("catalogo.txt", &texto)==0){
				error = LeerFicheroTexto(&texto, buffer_texto);
				f_close(&texto);

				cat = GenerarCatalogo(buffer_texto);
			}
			
		   }
               //Intriducir código LDC
           break;

           case 1: //Menu canciones USB - Como stop de cancion actual
               	//printf("Songs list\n");
        	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

               	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

		cat = MostrarCatalogo(cat);
		
		 
               //Intriducir código LDC y micro
           break;

           case 2: //Play cancion seleccionada
               //printf("Play song\n");
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

               	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

		if(nueva_cancion){
			if(AbrirFichero(cat -> nombre_mp4, &auidio)!=0 || AbrirFichero(cat -> nombre_txt, &texto)!=0){
				// Apertura correcta de fichero de audio y fichero de texto
				if(LeerFicheroTexto(&texto, buffer_texto) == 0)
					// Lee correctamente el fichero de texto
					
			}
			else
				estado = 3; // Estado de pausa para mostrar el mensaje de error
			break;	
		}
		if (ReproducirFicheroSonido() == 0){
			EnviarLetra(buffer_texto);
		}
               //Introducir código LDC y micro
           break;

           case 3: //Pause cancion seleccionada
               //printf("Pause song\n");
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
                 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

               HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
               //Introducir código LDC y micro
           break;

           default: //Volver a pulsar un botón, la has liado
           break;
       }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
