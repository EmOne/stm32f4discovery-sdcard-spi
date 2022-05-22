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
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "fatfs.h"
#include "i2c.h"
#include "rng.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fatfs_sd.h"
#include "string.h"
#include "stdio.h"
#include "mp3dec.h"
#include "Audio.h"
#include <string.h>
#include "i2s.h"

//#include "WiMODLRHCI.h"
//#include "WiMODLoRaWAN.h"
#include "WiMOD_LoRaWAN_API.h"
#include "WiMOD_HCI_Layer.h"
#include "SerialDevice.h"
#include "emod_uart.h"
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
FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
char buffer[1024]; //store data
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;
volatile uint32_t SD_Detect = 1;
volatile uint32_t USER_press = 0;

// Variables
//volatile uint32_t		time_var1, time_var2;
//USB_OTG_CORE_HANDLE		USB_OTG_Core;
//USBH_HOST				USB_Host;
//RCC_ClocksTypeDef		RCC_Clocks;
int			enum_done = 0;
volatile bool DMARunning;
// MP3 Variables
#define FILE_READ_BUFFER_SIZE 8192
MP3FrameInfo			mp3FrameInfo;
HMP3Decoder				hMP3Decoder;
FIL						file;
char					file_read_buffer[FILE_READ_BUFFER_SIZE];
volatile int			bytes_left;
static char					*read_ptr;
//TWiMODLORAWAN_ActivateDeviceData activationData;
#define UID_ADDR 0x1FFF7A10
const char appKey[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0f, 0x10 };
const char devEUI[8] = { 0x70, 0xB3, 0xD5, 0x8F, 0xFF, 0xFF, 0xFF, 0xFF };
const char appEUI[8] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22 };

const char NWKSKEY[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0f, 0x10 };
const char APPSKEY[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0f, 0x10 };
//TWiMODLRResultCodes hciResult;
UINT8 rspStatus;
//TWiMODLORAWAN_RadioStackConfig hciRadio;
//TWiMODLORAWAN_TX_Data lrwTxData;
LoRa_App loraAppStatus;
int __io_putchar(int ch);
uint8_t sensor_data[4];
//int __io_getchar(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
void send_uart (char* str);
int buf_size (char* buf);
void buf_clear (void);

// Private function prototypes
static int AudioCallback(void *context,int buffer);
static uint32_t Mp3ReadId3V2Tag(FIL* pInFile, char* pszArtist,
		uint32_t unArtistSize, char* pszTitle, uint32_t unTitleSize,
		char* pszContentType, uint32_t unContentTypeSize);
static void play_mp3(char* filename);
static FRESULT play_directory (const char* path, unsigned char seek);

//void JoinedNwkIndicationCallback(TWiMODLR_HCIMessage* rxMsg);
//
//void RxUDataIndicationCallback(TWiMODLR_HCIMessage* rxMsg);
//
//void RxCDataIndicationCallback(TWiMODLR_HCIMessage* rxMsg);
//
//void RxMacCmdIndicationCallback(TWiMODLR_HCIMessage* rxMsg);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void send_uart (char* str)
{
	int len = strlen(str);
	HAL_UART_Transmit(&huart2, (uint8_t *) str, len, 100);
}

int buf_size (char* buf)
{
	int i=0;
	while(*buf++ != '\0') i++;
	return i;
}

void buf_clear (void)
{
	for (int var = 0; var < sizeof(buffer); ++var) {
		buffer[var] = '\0';
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
  MX_USB_HOST_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_RNG_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* emWIMOD Intialize */
  HAL_GPIO_WritePin(WIMOD_RST_GPIO_Port, WIMOD_RST_Pin, GPIO_PIN_RESET);

  for (volatile int i = 0; i < 0x4fff; i++) {
	  __asm__ volatile("nop");
  }

  HAL_GPIO_WritePin(WIMOD_RST_GPIO_Port, WIMOD_RST_Pin, GPIO_PIN_SET);
#ifdef EM_WIMOD
  //  TWiMODLRHCI.begin(&huart3);
  //  WiMODLoRaWAN.Reset(&hciResult, &rspStatus);
  //  WiMODLoRaWAN.beginAndAutoSetup();
  //  activationData.DeviceAddress = WIMOD_DEV_ADDR;
  //  memcpy(activationData.NwkSKey, NWKSKEY, 16);
  //  memcpy(activationData.AppSKey, APPSKEY, 16);
  //  WiMODLoRaWAN.RegisterRxCDataIndicationClient(RxCDataIndicationCallback);
  //  WiMODLoRaWAN.RegisterRxUDataIndicationClient(RxUDataIndicationCallback);
  //  WiMODLoRaWAN.RegisterRxMacCmdIndicationClient(RxMacCmdIndicationCallback);
  //  WiMODLoRaWAN.RegisterJoinedNwkIndicationClient(JoinedNwkIndicationCallback);
  //  WiMODLoRaWAN.RegisterRxAckIndicationClient(NULL);
  //  WiMODLoRaWAN.RegisterTxUDataIndicationClient(NULL);
  //  WiMODLoRaWAN.RegisterTxCDataIndicationClient(NULL);
  //  WiMODLoRaWAN.RegisterNoDataIndicationClient(NULL);
  //  WiMODLoRaWAN.RegisterJoinTxIndicationClient(NULL);
  //  do{
  //	  WiMODLoRaWAN.Ping(&hciResult, &rspStatus);
  //  }  while (hciResult);
  //  WiMODLoRaWAN.DeactivateDevice(&hciResult, &rspStatus);
  //  hciRadio.BandIndex = LORAWAN_BAND_AS_923_TH_920;
  //  hciRadio.DataRateIndex = LORAWAN_DATA_RATE_AS923_LORA_SF10_125KHZ;
  //  hciRadio.TXPowerLevel = LORAWAN_TX_POWER_LEVEL_MAX;
  //  hciRadio.Retransmissions = 7;
  //  hciRadio.HeaderMacCmdCapacity = 15;
  //  hciRadio.PowerSavingMode = LORAWAN_POWER_SAVING_MODE_AUTO;
  //  hciRadio.Options = LORAWAN_STK_OPTION_ADR | LORAWAN_STK_OPTION_DUTY_CYCLE_CTRL |
  //		  LORAWAN_STK_OPTION_EXT_PKT_FORMAT |LORAWAN_STK_OPTION_DEV_CLASS_C;
  //  WiMODLoRaWAN.SetRadioStackConfig(&hciRadio, &hciResult, &rspStatus);
  //  WiMODLoRaWAN.ActivateDevice(&activationData, &hciResult, &rspStatus);

#else

  WiMOD_LoRaWAN_Init(&huart3);
  Ping();
  HAL_Delay(200);
  Deactivate();
  HAL_Delay(500);
  SetRadioStack();
  HAL_Delay(1000);
  loraAppStatus.devAddr.u32 = *((uint32_t *) UID_ADDR);
  memcpy(loraAppStatus.devEUI, devEUI, sizeof(loraAppStatus.devEUI));
  memcpy(&loraAppStatus.devEUI[4], &loraAppStatus.devAddr.u8, 4);
  memcpy(loraAppStatus.nwkSKey, NWKSKEY, sizeof(loraAppStatus.nwkSKey));
  memcpy(loraAppStatus.appSKey, APPSKEY, sizeof(loraAppStatus.appSKey));

  memcpy(loraAppStatus.appKey, appKey, sizeof(loraAppStatus.appKey));
  memcpy(loraAppStatus.appEUI, appEUI, sizeof(loraAppStatus.appEUI));

  ActivateABP();
  HAL_Delay(2000);
#endif

  /* Mount SD card */
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
  while (SD_Detect) {
	  HAL_Delay(500);
	  if(HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin) == GPIO_PIN_RESET) {
		  HAL_Delay(500);
		  SD_Detect = 0;
		  enum_done = 2;
	  }
  };
	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK) {
		send_uart("err: SD Card mounting failure.....\r\n");
		Error_Handler();
	} else {
		send_uart("info: SD Card mounted successfully.....\r\n");
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
	}

	/* Check free space */
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf(buffer, "info: SD Card total size:\t%lu\r\n", total);
	send_uart(buffer);
	buf_clear();
	free_space = (uint32_t) (fre_clust * pfs->csize * 0.5);
	sprintf(buffer, "info: SD Card free space size:\t%lu\r\n", free_space);
	send_uart(buffer);
//	/* Open file to write/ create a file if it doesn't exist */
//	fresult = f_open(&fil, "file1.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
//
//	/* Writing text */
//	sprintf(buffer, "This data is from the FILE1.txt. And it was written using ...f_puts... \r\n");
//	f_puts(buffer, &fil);
//
//	/* Close file */
//	fresult = f_close(&fil);
	InitializeAudio(Audio48000HzSettings);
	SetAudioVolume(50);
	AudioOn();

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    if (enum_done > 0) {
		play_directory("", 0);
		enum_done = 0;
	}
//    TWiMODLRHCI.Process();
//	if (USER_press) {
//		play_directory("", 0);
//		lrwTxData.Port = 2;
//		strcpy(lrwTxData.Payload, "Hello");
//		lrwTxData.Length = strlen(lrwTxData.Payload);
//		WiMODLoRaWAN.SendData(&lrwTxData, &hciResult, &rspStatus);
//	}

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 switch (GPIO_Pin) {
	 	case GPIO_PIN_0:
			if (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) {
				USER_press = 1;
				SendUData(2, sensor_data, 2);
			} else {
				USER_press = 0;
			}
	 		break;
		case GPIO_PIN_1:
			if(HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin) == GPIO_PIN_SET){
				SD_Detect = 1;
			}
			else{
				SD_Detect = 0;
			}
			break;
		default:
			break;
	 }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM3)
  {
	  HAL_GPIO_TogglePin(LD4_GPIO_Port, LD4_Pin);
  }
  else if(htim->Instance == TIM4)
  {
  }
}

const char *get_filename_ext(const char *filename) {
    const char *dot = strrchr(filename, '.');
    if(!dot || dot == filename) return "";
    return dot + 1;
}

static FRESULT play_directory (const char* path, unsigned char seek) {
	FRESULT res;
	FILINFO fno;
	DIR dir;
	char *fn; /* This function is assuming non-Unicode cfg. */
	char buffer[200];
#if _USE_LFN
	static TCHAR lfn[_MAX_LFN + 1];
	strcpy(fno.fname, lfn);
	fno.fsize = sizeof(lfn);
#endif

	res = f_opendir(&dir, path); /* Open the directory */
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno); /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0) break; /* Break on error or end of dir */
			if (fno.fname[0] == '.') continue; /* Ignore dot entry */
#if _USE_LFN
			fn = *fno.fname ? fno.fname : fno.fname;
#else
			fn = fno.fname;
#endif
//			if (fno.fattrib & AM_DIR) { /* It is a directory */
//
//			} else { /* It is a file. */
				sprintf(buffer, "%s/%02d.mp3", path, enum_done);
//				sprintf(buffer, "%s/%s", path, fn);
				send_uart("Playback: ");
				send_uart(buffer);

				// Check if it is an mp3 file
//				if (strcmp("mp3", get_filename_ext(buffer)) == 0) {
//
//					// Skip "seek" number of mp3 files...
//					if (seek) {
//						seek--;
//						continue;
//					}

//					InitializeAudio(Audio48000HzSettings);
//					SetAudioVolume(100);
//					AudioOn();

					play_mp3(buffer);

//					SetAudioVolume(0);
//					AudioOff();

					send_uart(" -> Done!!!");
					// Wait for user button release
//					while(!USER_press);
					HAL_Delay(1000);
					return res;
//				}
//				else
//				{
//					send_uart(" -> This file isn't MP3 file!!!");
//				}
				send_uart("\r\n");
//			}
		}
	}

	return res;
}

static void play_mp3(char* filename) {
	unsigned int br, btr;
	FRESULT res;

	bytes_left = FILE_READ_BUFFER_SIZE;
	read_ptr = file_read_buffer;

	if (FR_OK == f_open(&file, filename, FA_OPEN_EXISTING | FA_READ)) {

		// Read ID3v2 Tag
		char szArtist[120];
		char szTitle[120];
		char szContent[120];
		Mp3ReadId3V2Tag(&file, szArtist, sizeof(szArtist),
				szTitle, sizeof(szTitle), szContent, sizeof(szContent));

		// Fill buffer
		if(f_read(&file, file_read_buffer, FILE_READ_BUFFER_SIZE, &br) == FR_OK)
		{
			// Play mp3
			hMP3Decoder = MP3InitDecoder();
			AudioOn();
			PlayAudioWithCallback(AudioCallback, &file);

			for(;;) {
				/*
				 * If past half of buffer, refill...
				 *
				 * When bytes_left changes, the audio callback has just been executed. This
				 * means that there should be enough time to copy the end of the buffer
				 * to the beginning and update the pointer before the next audio callback.
				 * Getting audio callbacks while the next part of the file is read from the
				 * file system should not cause problems.
				 */
				if (bytes_left < (FILE_READ_BUFFER_SIZE / 2)) {

					if (read_ptr != NULL && bytes_left > 0) {
						// Copy rest of data to beginning of read buffer
						memcpy(file_read_buffer, read_ptr, bytes_left);
					}

					// Update read pointer for audio sampling
					read_ptr = file_read_buffer;

					// Read next part of file
					btr = FILE_READ_BUFFER_SIZE - bytes_left;
					res = f_read(&file, file_read_buffer + bytes_left, btr, &br);

					// Update the bytes left variable
					bytes_left = FILE_READ_BUFFER_SIZE;

					// Out of data or error ... Stop playback!
					if (br < btr || res != FR_OK || USER_press) {
//						AudioCallback(&file, 0);
//						StopAudio();
						while (DMARunning);
						// Close currently open file
						f_close(&file);

						// Return to previous function
						return;
					} else {
						PlayAudioWithCallback(AudioCallback, &file);
					}
				}

//				if (!DMARunning) {
//					if((file.fptr + bytes_left) < file.obj.objsize){
//						res = f_read(&file, file_read_buffer, bytes_left, &br);
//						// Update read pointer for audio sampling
//						read_ptr = file_read_buffer;
//						if (AudioCallback(&file, 0)) {
//							DMARunning = false;
//						}
//					} else {
////						StopAudio();
//						f_close(&file);
//						return;
//					}
//				} else {
//
//				}
			}
		} else {
			f_close(&file);
			send_uart("SD: Read error\r\n");
			return;
		}
	}
}

/*
 * Called by the audio driver when it is time to provide data to
 * one of the audio buffers (while the other buffer is sent to the
 * CODEC using DMA). One mp3 frame is decoded at a time and
 * provided to the audio driver.
 */
static int AudioCallback(void *context, int buffer) {
	static int16_t audio_buffer0[4096];
	static int16_t audio_buffer1[4096];

	char show_byte_left[32];
	int offset, err = 0;
	int outOfData = 0;

	FIL* f = (FIL*) context;
	int16_t *samples;

	if (buffer) {
		samples = audio_buffer0;
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
	} else {
		samples = audio_buffer1;
		HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	}

	offset = MP3FindSyncWord((unsigned char*)read_ptr, bytes_left);
	bytes_left -= offset;
	read_ptr += offset;

	err = MP3Decode(hMP3Decoder, (unsigned char**)&read_ptr, (int*)&bytes_left, samples, 0);

	if (err) {
		/* error occurred */
		switch (err) {
		case ERR_MP3_MAINDATA_UNDERFLOW:
//			/* do nothing - next call to decode will provide more mainData */
			break;
		case ERR_MP3_INDATA_UNDERFLOW:
		case ERR_MP3_FREE_BITRATE_SYNC:
		default:
			outOfData = 1;
			break;
		}
	}

	sprintf(show_byte_left, "\r\n remain %d bytes : %ld/%ld ", bytes_left, f->fptr - bytes_left, f->obj.objsize);
	send_uart(show_byte_left);

	if (!outOfData) {
		// no error
		MP3GetLastFrameInfo(hMP3Decoder, &mp3FrameInfo);

		// Duplicate data in case of mono to maintain playback speed
		if (mp3FrameInfo.nChans == 1) {
			for (int i = mp3FrameInfo.outputSamps; i >= 0; i--) {
				samples[2 * i] = samples[i];
				samples[2 * i + 1] = samples[i];
			}
			mp3FrameInfo.outputSamps *= 2;
		}
		if (mp3FrameInfo.samprate != hi2s3.Init.AudioFreq) {
			__HAL_I2S_DISABLE(&hi2s3);
			hi2s3.Init.AudioFreq =
					mp3FrameInfo.samprate != 0 ?
							mp3FrameInfo.samprate : I2S_AUDIOFREQ_DEFAULT;
			hi2s3.Instance = SPI3;
			hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
			hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
			hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
			hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
			hi2s3.Init.CPOL = I2S_CPOL_LOW;
			hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
			hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
			if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
				Error_Handler();
			}
			__HAL_I2S_ENABLE(&hi2s3);
		}
		if(mp3FrameInfo.outputSamps > 0)
			ProvideAudioBuffer(samples, mp3FrameInfo.outputSamps);
		else
			bytes_left = 0;
	}
	else
	{
		bytes_left = (bytes_left < (FILE_READ_BUFFER_SIZE / 2)) ? bytes_left : 0 ;
		sprintf(show_byte_left, "\r\n Out of data!!! err=%d", err);
		send_uart(show_byte_left);
	}
	return outOfData;
}

/*
 * Taken from
 * http://www.mikrocontroller.net/topic/252319
 */
static uint32_t Mp3ReadId3V2Text(FIL* pInFile, uint32_t unDataLen, char* pszBuffer, uint32_t unBufferSize)
{
	UINT unRead = 0;
	BYTE byEncoding = 0;
	if((f_read(pInFile, &byEncoding, 1, &unRead) == FR_OK) && (unRead == 1))
	{
		unDataLen--;
		if(unDataLen <= (unBufferSize - 1))
		{
			if((f_read(pInFile, pszBuffer, unDataLen, &unRead) == FR_OK) ||
					(unRead == unDataLen))
			{
				if(byEncoding == 0)
				{
					// ISO-8859-1 multibyte
					// just add a terminating zero
					pszBuffer[unDataLen] = 0;
				}
				else if(byEncoding == 1)
				{
					// UTF16LE unicode
					uint32_t r = 0;
					uint32_t w = 0;
					if((unDataLen > 2) && (pszBuffer[0] == 0xFF) && (pszBuffer[1] == 0xFE))
					{
						// ignore BOM, assume LE
						r = 2;
					}
					for(; r < unDataLen; r += 2, w += 1)
					{
						// should be acceptable for 7 bit ascii
						pszBuffer[w] = pszBuffer[r];
					}
					pszBuffer[w] = 0;
				}
			}
			else
			{
				return 1;
			}
		}
		else
		{
			// we won't read a partial text
			if(f_lseek(pInFile, f_tell(pInFile) + unDataLen) != FR_OK)
			{
				return 1;
			}
		}
	}
	else
	{
		return 1;
	}
	return 0;
}

/*
 * Taken from
 * http://www.mikrocontroller.net/topic/252319
 */
static uint32_t Mp3ReadId3V2Tag(FIL* pInFile, char* pszArtist, uint32_t unArtistSize,
		char* pszTitle, uint32_t unTitleSize, char* pszContentType, uint32_t unContentTypeSize)
{
	pszArtist[0] = 0;
	pszTitle[0] = 0;
	pszContentType[0] = 0;

	BYTE id3hd[10];
	UINT unRead = 0;
	if((f_read(pInFile, id3hd, 10, &unRead) != FR_OK) || (unRead != 10))
	{
		return 1;
	}
	else
	{
		uint32_t unSkip = 0;
		if((unRead == 10) &&
				(id3hd[0] == 'I') &&
				(id3hd[1] == 'D') &&
				(id3hd[2] == '3'))
		{
			unSkip += 10;
			unSkip = ((id3hd[6] & 0x7f) << 21) | ((id3hd[7] & 0x7f) << 14) | ((id3hd[8] & 0x7f) << 7) | (id3hd[9] & 0x7f);

			// try to get some information from the tag
			// skip the extended header, if present
			uint8_t unVersion = id3hd[3];
			if(id3hd[5] & 0x40)
			{
				BYTE exhd[4];
				f_read(pInFile, exhd, 4, &unRead);
				size_t unExHdrSkip = ((exhd[0] & 0x7f) << 21) | ((exhd[1] & 0x7f) << 14) | ((exhd[2] & 0x7f) << 7) | (exhd[3] & 0x7f);
				unExHdrSkip -= 4;
				if(f_lseek(pInFile, f_tell(pInFile) + unExHdrSkip) != FR_OK)
				{
					return 1;
				}
			}
			uint32_t nFramesToRead = 2;
			while(nFramesToRead > 0)
			{
				char frhd[10];
				if((f_read(pInFile, frhd, 10, &unRead) != FR_OK) || (unRead != 10))
				{
					return 1;
				}
				if((frhd[0] == 0) || (strncmp(frhd, "3DI", 3) == 0))
				{
					break;
				}
				char szFrameId[5] = {0, 0, 0, 0, 0};
				memcpy(szFrameId, frhd, 4);
				uint32_t unFrameSize = 0;
				uint32_t i = 0;
				for(; i < 4; i++)
				{
					if(unVersion == 3)
					{
						// ID3v2.3
						unFrameSize <<= 8;
						unFrameSize += frhd[i + 4];
					}
					if(unVersion == 4)
					{
						// ID3v2.4
						unFrameSize <<= 7;
						unFrameSize += frhd[i + 4] & 0x7F;
					}
				}

				if(strcmp(szFrameId, "TPE1") == 0)
				{
					// artist
					if(Mp3ReadId3V2Text(pInFile, unFrameSize, pszArtist, unArtistSize) != 0)
					{
						break;
					}
					nFramesToRead--;
				}
				else if(strcmp(szFrameId, "TIT2") == 0)
				{
					// title
					if(Mp3ReadId3V2Text(pInFile, unFrameSize, pszTitle, unTitleSize) != 0)
					{
						break;
					}
					nFramesToRead--;
				}
				else if(strcmp(szFrameId, "TCON") == 0)
				{
					// content type
					if(Mp3ReadId3V2Text(pInFile, unFrameSize, pszContentType, unContentTypeSize) != 0)
					{
						break;
					}
					nFramesToRead--;
				}
				else
				{
					if(f_lseek(pInFile, f_tell(pInFile) + unFrameSize) != FR_OK)
					{
						return 1;
					}
					nFramesToRead--;
				}
			}
		}
		if(f_lseek(pInFile, unSkip) != FR_OK)
		{
			return 1;
		}
	}

	return 0;
}

void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
	send_uart(" sent");
	AudioDMA_IRQHandler();
}
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s)
{
	Error_Handler();
}

//void JoinedNwkIndicationCallback(TWiMODLR_HCIMessage* rxMsg)
//{
//
//}
//
//void RxUDataIndicationCallback(TWiMODLR_HCIMessage* rxMsg)
//{
//
//}
//
//void RxCDataIndicationCallback(TWiMODLR_HCIMessage* rxMsg)
//{
//
//}
//
//void RxMacCmdIndicationCallback(TWiMODLR_HCIMessage* rxMsg)
//{
//
//}

int WiMOD_Error_Handler(void* obj)
{
	TWiMOD_HCI_Message*  rxMessage;
	if (obj != NULL) {
		rxMessage = obj;
		printf("ERR: SID:%d, MID:%d, s:%x\r\n", rxMessage->SapID, rxMessage->MsgID, rxMessage->State);
	}

}

int loraDataRx(uint8_t fport, uint8_t* data, size_t len)
{
    switch(fport) {
    case 99:
       //Restart system
      NVIC_SystemReset();
      break;

    case 3:
      if(len == 1) {
        //lora tx perior
//        sData_t.period =
    	  loraAppStatus.period = *data;
//        ev.commu = RUNNING;
      }
      break;
    case 0x12:
    	sensor_data[0] = data[0];
    	//Signal tower R/G
    	HAL_GPIO_WritePin(RELAY_1_GPIO_Port, RELAY_1_Pin, (sensor_data[0] >> 7) & 0x1);
    	HAL_GPIO_WritePin(RELAY_2_GPIO_Port, RELAY_2_Pin, (sensor_data[0] >> 6) & 0x1);
    	//Select message to play back
    	enum_done = sensor_data[0] & 0x3F;
    	break;
    case 0xFF:
      if(len > 0)
      {
        if(*data == 0x1)
        {
          printf("Acknowledges ResetInd command\r\n");
        }
      }
      break;
    default: break;
    };
    return 0;
}
/**
 * @brief		Trace output for standard output
 */
int fputc(int ch, FILE *f) {
  return ITM_SendChar(ch);
}

int __io_putchar(int ch) {
  return HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 500) == HAL_OK ? 0 : -1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart->Instance == USART2){
	 emod_RxCpltCallback(huart);
 } else if (huart->Instance == USART3){
	 USART_ITCharManager(huart);
	 WiMOD_LoRaWAN_Process();
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
  HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
  while (1)
  {
	  for (int var = 0; var < 10000000; ++var) {
		__NOP();
	  }
	  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
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
