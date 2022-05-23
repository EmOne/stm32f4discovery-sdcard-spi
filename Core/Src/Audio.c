#include "Audio.h"
#include "main.h"
#include "MY_CS43L22.h"
#include <stdlib.h>

#include "dma.h"
#include "i2s.h"
#include "i2c.h"

//static void WriteRegister(uint8_t address, uint8_t value);
static void StartAudioDMAAndRequestBuffers();
static void StopAudioDMA();

static AudioCallbackFunction *CallbackFunction;
static void *CallbackContext;
static int16_t * volatile NextBufferSamples;
static volatile int NextBufferLength;
static volatile int BufferNumber;
extern volatile bool DMARunning;
struct CS43_STATUS_T cs43_status = {0};

extern I2C_HandleTypeDef hi2c1;

extern I2S_HandleTypeDef hi2s3;

extern DMA_HandleTypeDef hdma_spi3_tx;

void InitializeAudio(int plln, int pllr, int i2sdiv, int i2sodd) {
	// Intitialize state.
	CallbackFunction = NULL;
	CallbackContext = NULL;
	NextBufferSamples = NULL;
	NextBufferLength = 0;
	BufferNumber = 0;
	DMARunning = false;

	MX_I2C1_Init();

		CS43_Init(&hi2c1, MODE_I2S);
		CS43_SetVolume(75);
		CS43_Enable_RightLeft(CS43_RIGHT_LEFT);

		__HAL_RCC_DMA1_CLK_ENABLE();

		MX_I2S3_Init();

		MX_DMA_Init();

	// Reset I2C.
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);
//
	// Configure I2C.
//	uint32_t pclk1 = 42000000;
//
//	I2C1 ->CR2 = pclk1 / 1000000; // Configure frequency and disable interrupts and DMA.
//	I2C1 ->OAR1 = I2C_OAR1_ADDMODE | 0x33;
//
//	// Configure I2C speed in standard mode.
//	const uint32_t i2c_speed = 100000;
//	int ccrspeed = pclk1 / (i2c_speed * 2);
//	if (ccrspeed < 4) {
//		ccrspeed = 4;
//	}
//	I2C1 ->CCR = ccrspeed;
//	I2C1 ->TRISE = pclk1 / 1000000 + 1;
//
//	I2C1 ->CR1 = I2C_CR1_ACK | I2C_CR1_PE; // Enable and configure the I2C peripheral.
//
//	// Configure codec.
//	WriteRegister(0x02, 0x01); // Keep codec powered off.
//	WriteRegister(0x04, 0xaf); // SPK always off and HP always on.
//
//	WriteRegister(0x05, 0x81); // Clock configuration: Auto detection.
//	WriteRegister(0x06, 0x04); // Set slave mode and Philips audio standard.
//
//	SetAudioVolume(0xff);
//
//	// Power on the codec.
//	WriteRegister(0x02, 0x9e);
//
//	// Configure codec for fast shutdown.
//	WriteRegister(0x0a, 0x00); // Disable the analog soft ramp.
//	WriteRegister(0x0e, 0x04); // Disable the digital soft ramp.
//
//	WriteRegister(0x27, 0x00); // Disable the limiter attack level.
//	WriteRegister(0x1f, 0x0f); // Adjust bass and treble levels.
//
//	WriteRegister(0x1a, 0x0a); // Adjust PCM volume level.
//	WriteRegister(0x1b, 0x0a);
//
//	// Disable I2S.
//	SPI3 ->I2SCFGR = 0;
//
//	// I2S clock configuration
//	RCC ->CFGR &= ~RCC_CFGR_I2SSRC; // PLLI2S clock used as I2S clock source.
//	RCC ->PLLI2SCFGR = (pllr << 28) | (plln << 6);
//
//	// Enable PLLI2S and wait until it is ready.
//	RCC ->CR |= RCC_CR_PLLI2SON;
//	while (!(RCC ->CR & RCC_CR_PLLI2SRDY ))
//		;
//
//	// Configure I2S.
//	SPI3 ->I2SPR = i2sdiv | (i2sodd << 8) | SPI_I2SPR_MCKOE;
//	SPI3 ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1
//			| SPI_I2SCFGR_I2SE; // Master transmitter, Phillips mode, 16 bit values, clock polarity low, enable.

}

void AudioOn() {
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	CS43_Start();

//	WriteRegister(0x02, 0x9e);
	SPI3 ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1
			| SPI_I2SCFGR_I2SE; // Master transmitter, Phillips mode, 16 bit values, clock polarity low, enable.
}

void AudioOff() {
	StopAudioDMA();
	CS43_Stop();
//	StopAudio();
//	WriteRegister(0x02, 0x01);
	SPI3 ->I2SCFGR = 0;
//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

}

void SetAudioVolume(int volume) {
	CS43_SetVolume(volume);
//	WriteRegister(0x20, (volume + 0x19) & 0xff);
//	WriteRegister(0x21, (volume + 0x19) & 0xff);
}

void OutputAudioSample(int16_t sample) {
//	while (!(SPI3 ->SR & SPI_SR_TXE ))
//		;
//	SPI3 ->DR = sample;
	HAL_I2S_Transmit_DMA(&hi2s3,(uint16_t *) &sample, 1);
}

void OutputAudioSampleWithoutBlocking(int16_t sample) {
	SPI3 ->DR = sample;
}

void PlayAudioWithCallback(AudioCallbackFunction *callback, void *context) {
//	StopAudioDMA();

	HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 0, 0);
	NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	SPI3 ->CR2 |= SPI_CR2_TXDMAEN; // Enable I2S TX DMA request.
//	HAL_I2S_DMAResume(&hi2s3);

	CallbackFunction = callback;
	CallbackContext = context;
	BufferNumber = 0;

	if (CallbackFunction) {
		if(CallbackFunction(CallbackContext, BufferNumber)){
//			DMARunning = false;
		}
	}
}

void StopAudio() {
	StopAudioDMA();
	SPI3 ->CR2 &= ~SPI_CR2_TXDMAEN; // Disable I2S TX DMA request.
	NVIC_DisableIRQ(DMA1_Stream7_IRQn);
	CallbackFunction = NULL;
//	HAL_I2S_DeInit(&hi2s3);
}

void ProvideAudioBuffer(void *samples, int numsamples) {
	while (!ProvideAudioBufferWithoutBlocking(samples, numsamples))
		__asm__ volatile ("wfi");
}

bool ProvideAudioBufferWithoutBlocking(void *samples, int numsamples) {
	if (NextBufferSamples)
		return false;

	NVIC_DisableIRQ(DMA1_Stream7_IRQn);

	NextBufferSamples = samples;
	NextBufferLength = numsamples;

	if (!DMARunning) {
		StartAudioDMAAndRequestBuffers();
	}

	NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	return true;
}

//static void WriteRegister(uint8_t address, uint8_t value) {
//	while (I2C1 ->SR2 & I2C_SR2_BUSY )
//		;
//
//	I2C1 ->CR1 |= I2C_CR1_START; // Start the transfer sequence.
//	while (!(I2C1 ->SR1 & I2C_SR1_SB ))
//		; // Wait for start bit.
//
//	I2C1 ->DR = 0x94;
//	while (!(I2C1 ->SR1 & I2C_SR1_ADDR ))
//		; // Wait for master transmitter mode.
//	I2C1 ->SR2;
//
//	I2C1 ->DR = address; // Transmit the address to write to.
//	while (!(I2C1 ->SR1 & I2C_SR1_TXE ))
//		; // Wait for byte to move to shift register.
//
//	I2C1 ->DR = value; // Transmit the value.
//
//	while (!(I2C1 ->SR1 & I2C_SR1_BTF ))
//		; // Wait for all bytes to finish.
//	I2C1 ->CR1 |= I2C_CR1_STOP; // End the transfer sequence.
//}

static void StartAudioDMAAndRequestBuffers() {
	// Configure DMA stream.
//	__HAL_I2S_ENABLE(&hi2s3);
	DMA1_Stream7 ->CR = (0 * DMA_SxCR_CHSEL_0 ) | // Channel 0
			(1 * DMA_SxCR_PL_0 ) | // Priority 1
			(1 * DMA_SxCR_PSIZE_0 ) | // PSIZE = 16 bit
			(1 * DMA_SxCR_MSIZE_0 ) | // MSIZE = 16 bit
			DMA_SxCR_MINC | // Increase memory address
			(1 * DMA_SxCR_DIR_0 ) | // Memory to peripheral
			DMA_SxCR_TCIE; // Transfer complete interrupt
	DMA1_Stream7 ->NDTR = NextBufferLength;
	DMA1_Stream7 ->PAR = (uint32_t) &SPI3 ->DR;
	DMA1_Stream7 ->M0AR = (uint32_t) NextBufferSamples;
	DMA1_Stream7 ->FCR = DMA_SxFCR_DMDIS;
	DMA1_Stream7 ->CR |= DMA_SxCR_EN;
	SPI3 ->CR2 |= SPI_CR2_TXDMAEN; // Enable I2S TX DMA request.

	// Update state.
	NextBufferSamples = NULL;
	BufferNumber ^= 1;
	DMARunning = true;

	// Invoke callback if it exists to queue up another buffer.
	if (CallbackFunction){
		if(CallbackFunction(CallbackContext, BufferNumber)){
//			NextBufferSamples = NULL;
//			DMARunning = false;
		}
	}
}

static void StopAudioDMA() {
	DMA1_Stream7 ->CR &= ~DMA_SxCR_EN; // Disable DMA stream.
	while (DMA1_Stream7 ->CR & DMA_SxCR_EN ); // Wait for DMA stream to stop.
	DMARunning = false;
}

void AudioDMA_IRQHandler() {
	DMA1 ->HIFCR |= DMA_HIFCR_CTCIF7; // Clear interrupt flag.
//	send_uart(" sent");
	if (NextBufferSamples) {
		StartAudioDMAAndRequestBuffers();
	} else {
		DMARunning = false;
	}
}
