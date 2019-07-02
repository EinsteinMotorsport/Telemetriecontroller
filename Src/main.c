/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMBER_OF_DATA_PUFFER 8 //NUMBER_OF_DATA_PUFFER gibt an, wie viele Arrays als Datenpuffer angelegt werden. Ein Datenpuffer hat die
																//Laenge von 8, da eine CAN-Nachricht 8 Datenbyte hat.
#define LENGTH_OF_PACKED_DATA 132 //LENGTH_OF_PACKED_DATA gibt an, wie lang ein Array als Datenpuffer ist. 128 Datenbyte + 2 Statusbytes 
																	//Funkmodul + 1 Byte Length + 1 Byte CRC = 132
#define HIGHEST_WRITE_POSITION 122 //HIGHEST_WRITE_POSITION gibt die groesstmoegliche Position an, ab dem keine neue CAN-Nachrichten in 
																	 //das Datenpaket fuer das Funkmodul passt.
#define NUMBER_OF_IDS 30 //NUMBER_OF_IDS gibt die Anzahl an IDs an, die im MS6 konfiguriert sind
#define OFFSET_OF_LOWEST_ID 0x10 //OFFSET_OF_LOWEST_ID entspricht dem Wert der kleinsten ID. Die Konstante wird im Programm verwendet, um die CAN-Nachrichten zu filtern.
#define WAIT_TIME 1000 //WAIT_TIME gibt die Zeit in Millisekunden an, die der CAN-Empfangs-Thread pausiert wird, wenn eine bestimme Anzahl
											//an Datenpaketen (NUMBER_OF_RADIO_MODULE_MESSAGES_UNTIL_PAUSE) an das Funkmodul gesendet wurde.
#define NUMBER_OF_RADIO_MODULE_MESSAGES_UNTIL_PAUSE 1000 //NUMBER_OF_RADIO_MODULE_MESSAGES_UNTIL_PAUSE gibt die Anzahl an eingetroffenen CAN-Nachrichten an, 
																												 //nach denen eine Pause gestartet wird.
#define NUMBER_OF_CONFIG_BYTES 4//NUMBER_OF_CONFIG_BYTES gibt an, wie viele Datenbyte das Paket enthaelt, das fuer die Konfiguration des CAN-Filters vom Laptop
																//an das Funkmodul gesendet wird.
#define START_BYTE_OF_CONFIGURATION_PACKET 0xDD //START_BYTE_OF_CONFIGURATION_PACKET entspricht dem ersten Byte des Konfigurationspakets fuer die CAN-Filterung.
#define NUMBER_OF_BYTES_CONFIGURATION_PACKET 4 //NUMBER_OF_BYTES_CONFIGURATION_PACKET enthaelt die Anzahl an Bytes, das ein Konfigurationspaket fuer die CAN-Filterung enthaelt

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

IWDG_HandleTypeDef hiwdg;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
bool packedDataWasSent; //packedDataWasSent wird verwendet, um zu speichern, ob ein Datenpaket an das Funkmodul gesendet wurde
uint8_t packedData[8][132]; //packedData[NUMBER_OF_DATA_PUFFER][LENGTH_OF_PACKED_DATA] enthaelt die Nutzdaten, die ueber das Funkmodul versendet werden. Der Datenpuffer besteht
														//aus 8 Arrays. Zuerst wird ein Datenpuffer beschrieben. Ist dieser voll, wird dieser Datenpuffer ueber UART an das Funkmodul versendet.
														//Um die Nutzdaten waehrend des Sendevorgangs nicht zu ueberschreiben, werden die nachfolgenden CAN-Nachrichten in den naechsten Datenpuffer geschrieben.
														
uint8_t positionPackedData; //positionPackedData wird verwendet, um die aktuelle Schreibposition in packedData zu speichern
uint8_t selectedPackedData;	//selectedPackedData wird verwendet, um den aktuell ausgewaehlten Datenpuffer zu speichern
uint8_t checksum; //checksum wird verwendet, um die berechnete Checksumme des Datenpakets zu speichern
uint8_t rxDataCan[8]; //rxDataCan wird verwendet, um die Nachrichten aus dem CAN-Eingangspuffer zu speichern
uint32_t numberCanMessages; //numberCanMessages wird verwendet, um die Anzahl der Nachrichten im CAN-Eingangspuffer zu speichern
uint32_t counterSendPackages; //counterSendPackages zaehlt die ueber UART versendeten Datenpakete

bool saveConfigPacketForCanFiltering; //configPacketForCanFilteringStarted gibt an, ob das erste Byte des Konfigurationspakets fuer die CAN-Filterung empfangen wurde
uint8_t counterBytesConfigPacket; //counterBytesConfigPacket gibt an, wie viele Bytes des Konfigurationspakets fuer die CAN-Filterung empfangen wurde

uint8_t uartReceiveData[1]; //uartReceiveData wird verwendet, um die ueber UART eintreffenden Daten zu speichern

bool idsToSend[30]; //idsToSend wird verwendet, um zu speichern, welche IDs an das Funkmodul weitergeleitet werden sollen
uint8_t idsToSendPosition; //idsToSendPosition wird verwendet, um die akutelle Schreibposition im Array idsToSend zu speichern
uint8_t idsToSendConfigPacket[4]; //idsToSendConfigPacket wird verwendet, um die ueber das Funkmodul eintreffenden Konfigurationspakete zu speichern

CAN_RxHeaderTypeDef rxHeader; //rxHeader wird verwendet, um die Statusinformationen der Nachrichten aus dem CAN-Eingangspuffer zu speichern

osThreadId receiveCanDataThreadHandle; //receiveCanDataThreadHandle wird verwendet, um den Thread, der die CAN-Nachrichten empfaengt, zu konfigurieren
osThreadId saveCanMessagesThreadHandle;  //saveCanMessagesThreadHandle wird verwendet, um den Thread, der die CAN-Nachrichten auf die SD-Karte schreibt, zu konfigurieren

FATFS SdFatFs; //SdFatFs wird zum Einbinden der SD-Karte benoetigt.
FIL file; //file ist der Zeiger auf die Datei, in die gerade auf der SD-Karte geschrieben wird.
char SdPath[4]; //SdPath enthaelt Informationen fuer den Controller, wie die SD-Karte eingebunden worden ist.
FRESULT fres; //fres enthaelt nach einem Schreibvorgang auf die SD-Karte, ob dieser erfolgreich war oder nicht
uint32_t counterBytesWritten; //counterBytesWritten enthaelt nach einem Schreibvorgang die Anzahl der geschriebenen Zeichen

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_SDIO_SD_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */


void PackData(uint8_t[], uint32_t canId); //PackData verpackt die ankommenden CAN-Nachrichten in das Datenformat fuer das Funkmodul
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart); //HAL_UART_RxCpltCallback wird ueberschrieben, damit bei jedem Empfangsinterrupt die Methode ausgefuehrt wird.
void InitSdCard(); //InitSdCard verbindet die SD-Karte mit dem Controller.
void SaveDataOnSdCard(uint8_t *data); //SaveDataOnSdCard schreibt Daten auf die SD-Karte.

static void ReceiveCanDataThread(void const * argument); //ReceiveCanDataThread realisiert den Thread, in dem die CAN-Nachrichten empfangen werden
static void SaveCanMessagesThread(void const * argument); //SaveCanMessagesThread realisiert den Thread, in dem die CAN-Nachrichten auf die SD-Karte geschrieben werden
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
  MX_CAN1_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  MX_SDIO_SD_Init();
  /* USER CODE BEGIN 2 */

	//InitSdCard initialsiert die SD-Karte und bindet sie in das System ein.
	//InitSdCard();

	//Hier werden die Variablen mit ihren Anfangswerten initialisiert.
	packedDataWasSent = true; //Es wurde zwar noch kein Datenpaket ueber UART versendet, aber packedDataWasSent wird trozdem auf true gesetzt, da sonst das Paket wegen der if-Abfrage
														//spaeter nicht initialisert werden wuerde.
	selectedPackedData = 0; //Zu Beginn wird der erste Datenpuffer ausgewaehlt (Das erste Feld des mehrdimensionalen Arrays packedData).
	checksum = 0x00; //Zu Beginn ist die Checksumme gleich 0.
	positionPackedData = 0; //Zu Beginn ist die Schreibposition im Datenpuffer packedData gleich 0.
	counterSendPackages = 0; //Zu Beginn ist die Anzahl an gesendeten Datenpakten gleich 0.
	saveConfigPacketForCanFiltering = false; //Zu Beginn wurde noch kein Konfigurationspaket empfangen.
	counterBytesConfigPacket = 0; //Zu Beginn wurden noch keine Konfigurationspaketbytes empfangen.
	
	//Alle Datenpuffer werden mit 0 initialisiert.
	for(int i = 0; i < NUMBER_OF_DATA_PUFFER; i++)
	{
		for(int k = 0; k < LENGTH_OF_PACKED_DATA; k++)
		{
			packedData[i][k] = 0x00;
		}
	}
	
	//Hier wird festgelegt, welche IDs standardmaessig an das Funkmodul weitergeleitet werden.
	for(int i = 0; i < NUMBER_OF_IDS; i++)
	{
		idsToSend[i] = false;
	}
	
	idsToSend[0] = true;  //ID 0x10 wird standardmaessig gesendet
	idsToSend[1] = true;  //ID 0x11 wird standardmaessig gesendet
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
	
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	
	//osThreadDef konfiguriert einen neuen Thread mit dem Namen "ReceiveCanData" an. Jedesmal, wenn der Thread gestartet wird, wird die Funktion "ReceiveCanDataThread" ausgefuehrt.
	//Die Prioritaet des Threads ist "Normal". Die Anzahl an Instanzen ist "0". Der Thread hat die Stack-Groesse von "configMINIMAL_STACK_SIZE".
	osThreadDef(ReceiveCanData, ReceiveCanDataThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	
	//osThreadCreate legt einen neuen Thread des Typs "ReceiveCanData" an. Es werden am Anfang keine Parameter uebergeben, deshalb "NULL".
	//Die Funktion liefert als Rueckgabewert einen Thread-Handler, der in der Variablen receiveCanDataThreadHandle abgespeichert wird.
	receiveCanDataThreadHandle = osThreadCreate(osThread(ReceiveCanData), NULL);
	
	//osThreadDef konfiguriert einen neuen Thread mit dem Namen "SaveCanMessages" an. Jedesmal, wenn der Thread gestartet wird, wird die Funktion "SaveCanMessagesThread" ausgefuehrt.
	//Die Prioritaet des Threads ist "Normal". Die Anzahl an Instanzen ist "0". Der Thread hat die Stack-Groesse von "configMINIMAL_STACK_SIZE".
	//osThreadDef(SaveCanMessages, SaveCanMessagesThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
	
	//osThreadCreate legt einen neuen Thread des Typs "SaveCanMessages" an. Es werden am Anfang keine Parameter uebergeben, deshalb "NULL".
	//Die Funktion liefert als Rueckgabewert einen Thread-Handler, der in der Variablen saveCanMessagesThreadHandle abgespeichert wird.
	//saveCanMessagesThreadHandle = osThreadCreate(osThread(SaveCanMessages), NULL);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

	//Berechnung Prescaler und TimeQuanta
	//fAPB1 / CanDatenrate = (1 + TimeSeg1 + TimeSeg2) * Prescaler
	//fAPB1 in STM32CubeMX in Clock Configuration unter APB1 peripheral clocks (MHz) ablesen
	//Bei CanDatenrate die gewuenschte Datenrate einsetzen
	//Prescaler waehlen (muss zwischen 1 und 1024 liegen)
	//TimeSeg1 (muss zwischen 1 und 16 liegen) und TimeSeg2 (muss zwischen 1 und 8 liegen) so waehlen, dass die Gleichung erfuellt ist
	//TimeSeg2 sollte etwas groesser als TimeSeg1 sein
	
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	//set interrupt mode for receiving new can data byte into rxfifo0
	//SET_BIT(hcan1.Instance->IER, CAN_IER_FMPIE0);
	
	CAN_FilterTypeDef sFilterConfig;
	
	//Configure CAN filter
	sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  
  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }
	
	//Start CAN peripheral
	
	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

	//Berechnung Prescaler und Reload
	//fIWDG * ZeitintervallWatchDog = Prescaler * Reload
	//fIWDG in STM32CubeMX in Clock Configuration unter To IWDG (KHz) ablesen
	//ZeitintervallWatchDog ist die Zeit, nach der ein Reset des Controllers durchgefuhert wird, wenn der Watchdog nicht zurueckgesetzt wird
	//Prescaler (kann nur die Werte 4, 8, 16, 32, 64, 128 oder 256 annehmen) und Reload (muss zwischen 0 und 4095 liegen) so waehlen, dass Gleichung erfuellt ist
	
  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 2500;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
	
  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

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

	//__HAL_UART_ENABLE_IT setzt den Empfangsmodus von USART3 auf Interrupt
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, LED_Red_Pin|LED_Blue_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Red_Pin LED_Blue_Pin */
  GPIO_InitStruct.Pin = LED_Red_Pin|LED_Blue_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_Card_Platform_Settings_Pin */
  GPIO_InitStruct.Pin = SD_Card_Platform_Settings_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_Card_Platform_Settings_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/**
  *@brief HAL_UART_RxCpltCallback stellt den UART-Emfpangs-Interrupt dar. Die Methode wird jedesmal ausgefuehrt, wenn auf dem UART-Bus ein Byte eintrifft.
  *@param UART_HandleTypeDef *huart ist der UART-Handler.
  *@retval Keiner
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	
	//Wenn die ueber UART empfangenen Bytes gespeichert werden sollen, liefert if true.
	if(saveConfigPacketForCanFiltering == true)
	{
		//In der Variablen idsToSendConfigPacket wird an der Stelle counterBytesConfigPacket das ueber UART empfangene Byte des Konfigurationspaket gespeichert.
		idsToSendConfigPacket[counterBytesConfigPacket] = uartReceiveData[0];
		//Es wurde ein Byte des Konfigurationspakets empfangen, deshalb wird der Zaehler um eins erhoeht. 
		counterBytesConfigPacket++;
		
		//Falls alle Bytes des Konfigurationspakets empfangen wurden, liefert if true.
		if(counterBytesConfigPacket == NUMBER_OF_BYTES_CONFIGURATION_PACKET)
		{
			//Da nun keine weiteren Bytes des Konfigurationspakets empfangen werden sollen, wird die Variable saveConfigPacketForCanFiltering wieder auf false gesetzt.
			saveConfigPacketForCanFiltering = false;
			//Um wieder neue Konfigurationspakete empfangen zu koennen, wird der Zaehler wieder auf 0 gesetzt.
			counterBytesConfigPacket = 0;
			
			for(int i = 0; i < NUMBER_OF_BYTES_CONFIGURATION_PACKET; i++)
			{
				for(int k = 0; k < 8; k++)
				{
					idsToSend[idsToSendPosition] = (bool)((idsToSendConfigPacket[i] >> k) & 1);
					idsToSendPosition++;
				}
			}
		}	
	}
	
	//Falls das ueber UART empfangene Byte dem ersten Byte des Konfigurationspakets entspricht, liefert if true.
	if(uartReceiveData[0] == START_BYTE_OF_CONFIGURATION_PACKET)
	{
		//Da nun die weiteren Bytes des Konfigurationspakets empfangen werden sollen, wird die Variable saveConfigPacketForCanFiltering auf true gesetzt.
		saveConfigPacketForCanFiltering = true;
		
		//Die aktuelle Schreibposition im Array wird auf 0 gesetzt.
		idsToSendPosition = 0;
	}
}

/**
	* @brief PackData packt die einzelnen CAN-Nachrichten zu einem groesseren Datenpaket zusammen, das ueber UART an das Funkmodul geschickt wird
	* @param uint8_t data[] enthaelt die Nutzdaten einer CAN-Nachricht
	* @param uint32_t canId enthaelt die CAN-Standard-ID zum Datenpaket uint8_t data[]
  * @retval Keiner
*/
void PackData(uint8_t data[], uint32_t canId)
{
	//Falls ein Datenpaket ueber UART versendet wurde, werden die Variablen wieder mit ihren Standardwerten initialisiert.
	if(packedDataWasSent == true)
	{
		//Um alte Daten zu ueberschreiben, wird der Datenpuffer mit lauter 0en initialisiert.
		for(int i = 0; i < LENGTH_OF_PACKED_DATA; i++)
		{
			packedData[selectedPackedData][i] = 0x00;
		}
		
		//Die Schreibposition des Datenpuffers wird auf 0 gesetzt.
		positionPackedData = 0;
		//Die Checksumme wird auf 0 gesetzt.
		checksum = 0x00;
		
		//Hier werden die Statusbits, die fuer das Funkmodul notwendig sind, gesetzt. Da die Laenge der Nutzdaten in einem Datenpaket immer 128 Byte sind, wird das Lenght-Byte
		//des Funkmoduls immer mit 0x80 initialisiert.
		packedData[selectedPackedData][0] = 0x02;
		packedData[selectedPackedData][1] = 0x00;
		packedData[selectedPackedData][2] = 0x80;
		//Es wurden drei Byte in packedData geschrieben, deshalb hat sich die Schreibposition positionPackedData um += 3 veraendert.
		positionPackedData += 3;
		
		//Die Variable packedDataWasSent wird wieder auf false gesetzt.
		packedDataWasSent = false;
	}
	
	//Da die CAN-ID 32 bit lang ist, aber die letzten beiden Byte immer 0 sind, werden nur die ersten beiden Byte verwendet.
	//An den Datenpuffer packedData werden die beiden Byte der Standard-ID angehaengt.
	//packedData[selectedPackedData][positionPackedData] = (canId >> 8) & 0xFF;
	//positionPackedData++;
	packedData[selectedPackedData][positionPackedData] = canId & 0xFF;
	positionPackedData++;

	
	//An den Datenpuffer werden die Nutzbytes angehaengt.
	//Da jedes CAN-Paket 8 Byte hat, werden immer 8 Byte angehaengt.
	
	for(int i = 0; i < 8; i++)
	{
		packedData[selectedPackedData][positionPackedData] = data[i];
		positionPackedData++;
	}

	
	//Falls die Schreibposition im Datenpuffer groesser gleich als 121 ist, ist kein Platz mehr fuer eine weiter CAN-Nachricht. Deshalb wird die Checksumme berechnet
	//und das Paket ueber UART an das Funkmodul versendet.
	if(positionPackedData >= HIGHEST_WRITE_POSITION)
	{
		//Die Checksummenberechnung ist eine XOR-Verknuepfung aller Bytes im Datenpaket.
		for(int i = 0; i < LENGTH_OF_PACKED_DATA - 1; i++)
		{
			checksum ^= packedData[selectedPackedData][i];
		}
		
		//Die berechnete Checksumme wird an der letzten Stellen des Datenpakets angefuegt.
		packedData[selectedPackedData][LENGTH_OF_PACKED_DATA - 1] = checksum;
		
		//Das Datenpaket wird ueber UART an das Funkmodul versendet.
		HAL_UART_Transmit_IT(&huart3, packedData[selectedPackedData], sizeof(packedData[selectedPackedData]));
		
		//Der Datenpuffer-Counter wird um eins erhoeht. 
		selectedPackedData++;
		
		//Da nur 8 Datenpuffer vorhanden sind, wird der Datenpuffer-Counter selectedPackedData bei 8 wieder auf 0 gesetzt.
		if(selectedPackedData == NUMBER_OF_DATA_PUFFER)
		{
			selectedPackedData = 0;
		}
		
		//Da das Datenpaket versendet wurde, wird packedDataWasSent auf true gesetzt.
		packedDataWasSent = true;
		//Da ein weiteres Datenpaket versendet wurde, wird der Zaehler der versendeten Nachrichten um eins erhoeht.
		counterSendPackages++;
	}
}

/**
	* @brief ReceiveCanDataThread ist die Funktion, die aufgerufen wird, wenn der Thread ReceiveCanData gestartet wird. 
  *        Die Funktion packt die einzelnen CAN-Nachrichten zu einem groesseren Datenpaket zusammen, das ueber UART an das Funkmodul geschickt wird
	* @param argument enthaelt den Startwert, der beim Erstellen des Threads an die Funktion uebergeben wird. 
  * @retval Keiner
*/
void ReceiveCanDataThread(void const * argument)
{
	for(;;)
	{
		//HAL_CAN_GetRxFifoFillLevel liest die Anzahl an Nachrichten aus, die im CAN-Eingangspuffer CAN_RX_FIFO0 gespeichert sind und speichert sie in numberCanMessages ab
		numberCanMessages = HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0);
		
		//Jenachdem, wie viele Nachrichten im CAN-Eingangspuffer gespeichdert sind, wird die Laenge der for-Schleife veraendert
		for(int i = 0; i < numberCanMessages; i++)
		{
			HAL_IWDG_Refresh(&hiwdg);
			//HAL_CAN_GetRxMessage versucht, eine Nachricht aus dem CAN-Eingangspuffer CAN_RX_FIFO0 zu lesen. 
			//Schlaegt der Lesevorgang fehl, liefert die Funktion als Rueckgabewert HAL_ERROR und die if-Bedingung wird nicht ausgefuehrt
			//Gelingt der Lesevorgang, liefert die Funktion als Rueckgabewert HAL_OK. Die Statusinformationen der CAN-Nachricht werden in rxHeader gespeichert.
			//Die Nutzdaten der CAN-Nachricht werden in rxDataCan gespeichert.
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxDataCan) == HAL_OK)
			{				
				//Die eintreffenden CAN-Nachrichten werden nach ihrem Identifier gefiltert. Soll die ID an das Funkmodul weitergeleitet werden, liefert das if true.
				if(idsToSend[rxHeader.StdId - OFFSET_OF_LOWEST_ID] == true)
				{					
					//Die Standard-ID der CAN-Nachricht und die Nutzdaten werden an die Funktion PackData uebergeben.
					PackData(rxDataCan, rxHeader.StdId);
				}
			}
		}
		
		//Wenn eine bestimmte Anzahl an Nachrichten versendet wurde (NUMBER_OF_CAN_MESSAGES_UNTIL_PAUSE), liefert das if true.
		if(counterSendPackages > NUMBER_OF_RADIO_MODULE_MESSAGES_UNTIL_PAUSE)
		{
			osDelay(WAIT_TIME);
			counterSendPackages = 0;
		}
	}
}


/**
	* @brief InitSdCard initialisiert die SD-Karte und bindet sie in das System ein.
	* @param Keiner
  * @retval Keiner
*/
void InitSdCard()
{
	//FATFS_LinkDriver verbindet den Controller mit der SD-Karte
	if(FATFS_LinkDriver(&SD_Driver, SdPath) == 0)
	{
		//f_mount bindet die SD-Karte in das System ein
		if(f_mount(&SdFatFs, (TCHAR const*)SdPath, 0) == FR_OK)
		{
			HAL_GPIO_TogglePin(LED_Blue_GPIO_Port, LED_Blue_Pin);
		}
	}
}


/**
	* @brief ReceiveCanDataThread ist die Funktion, die aufgerufen wird, wenn der Thread ReceiveCanData gestartet wird. 
  *        Die Funktion packt die einzelnen CAN-Nachrichten zu einem groesseren Datenpaket zusammen, das ueber UART an das Funkmodul geschickt wird
	* @param uint8_t *data enthaelt den Text, der in die Datei auf der SD-Karte geschrieben werden soll.
  * @retval Keiner
*/
void SaveDataOnSdCard(uint8_t *data)
{
	//f_open oeffnet die Datei mit dem Namen "Datei.txt" und schreibt an deren Ende weiter (FA_OPEN_APPEND). Der Zeiger auf die Datei wird in file gespeichert.
	if(f_open(&file, "Datei.txt", FA_OPEN_APPEND) == FR_OK)
	{
		//f_write schreibt den Text in data in die Datei. In counterBytesWritten steht nach dem Schreibvorgang die Anzahl der geschriebenen Zeichen. Der 
		//Rueckgabewert der Methode gibt an, ob der Schreibvorgang erfolgreich war oder nicht.
		fres = f_write(&file, &data, sizeof(&data), (void *)&counterBytesWritten);
		
		//Falls keine Zeichen geschrieben wurden oder der Schreibvorgang fehlgeschlagen ist, liefert das if true.
		if((counterBytesWritten == 0) || (fres != FR_OK))
		{
			HAL_GPIO_TogglePin(LED_Red_GPIO_Port, LED_Red_Pin);
		}
		else
		{
			//f_close schliesst die Datei. Wird die Datei nicht geschlossen und die SD-Karte einfach so entnommen, kann es sein, dass nicht der ganze Schreibpuffer
			//in die Datei geschrieben worden ist und dadurch Daten verloren gehen.
			f_close(&file);
		}
	}
	
	//FATFS_UnLinkDriver wirft die SD-Karte aus. 
	FATFS_UnLinkDriver(SdPath);
}

void SaveCanMessagesThread(void const * argument)
{
	
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for FATFS */
  //MX_FATFS_Init();

  /* USER CODE BEGIN 5 */
	//Die Methode MX_FATFS_Init() sollte immer auskommentiert werden. Die Initialisierung der SD-Karte wird in einer eigenen Methode durchgefuehrt.
  /* Infinite loop */
  for(;;)
  {
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
