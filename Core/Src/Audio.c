#include "Audio.h"
#include "main.h"

#include <stdlib.h>

//static void WriteRegister(uint8_t address, uint8_t value);
static void StartAudioDMAAndRequestBuffers();
static void StopAudioDMA();

//static uint8_t iData[2];
static AudioCallbackFunction *CallbackFunction;
static void *CallbackContext;
static int16_t * volatile NextBufferSamples;
static volatile int NextBufferLength;
static volatile int BufferNumber;
static volatile bool DMARunning;

extern I2C_HandleTypeDef hi2c1;
extern I2S_HandleTypeDef hi2s3;

/* Initial Volume level (from 0 (Mute) to 100 (Max)) */
static uint8_t Volume = 70;

void InitializeAudio(int plln, int pllr, int i2sdiv, int i2sodd) {
	// Intitialize state.
	CallbackFunction = NULL;
	CallbackContext = NULL;
	NextBufferSamples = NULL;
	NextBufferLength = 0;
	BufferNumber = 0;
	DMARunning = false;

	// Reset the codec.
	HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_RESET);
	for (volatile int i = 0; i < 0x4fff; i++) {
		__asm__ volatile("nop");
	}
	HAL_GPIO_WritePin(Audio_RST_GPIO_Port, Audio_RST_Pin, GPIO_PIN_SET);

	BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, Volume, AUDIO_FREQUENCY_48K);
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

	// Configure codec.
//	WriteRegister(CS43L22_REG_POWER_CTL1, 0x01); // Keep codec powered off.
//	WriteRegister(CS43L22_REG_POWER_CTL2, 0xaf); // SPK always off and HP always on.
//
//	WriteRegister(CS43L22_REG_CLOCKING_CTL, 0x81); // Clock configuration: Auto detection.
//	WriteRegister(CS43L22_REG_INTERFACE_CTL1, 0x04); // Set slave mode and Philips audio standard.
//
//	SetAudioVolume(0xAF);
//
//	// Power on the codec.
//	WriteRegister(CS43L22_REG_POWER_CTL1, 0x9e);
//
//	// Configure codec for fast shutdown.
//	WriteRegister(CS43L22_REG_ANALOG_ZC_SR_SETT, 0x00); // Disable the analog soft ramp.
//	WriteRegister(CS43L22_REG_MISC_CTL, 0x04); // Disable the digital soft ramp.
//
//	WriteRegister(CS43L22_REG_LIMIT_CTL1, 0x00); // Disable the limiter attack level.
//	WriteRegister(CS43L22_REG_TONE_CTL, 0x0f); // Adjust bass and treble levels.
//
//	WriteRegister(CS43L22_REG_PCMA_VOL, 0x0a); // Adjust PCM volume level.
//	WriteRegister(CS43L22_REG_PCMB_VOL, 0x0a);


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
	BSP_AUDIO_OUT_Resume();
//	WriteRegister(CS43L22_REG_POWER_CTL1, 0x9e);
//	SPI3 ->I2SCFGR = SPI_I2SCFGR_I2SMOD | SPI_I2SCFGR_I2SCFG_1
//			| SPI_I2SCFGR_I2SE; // Master transmitter, Phillips mode, 16 bit values, clock polarity low, enable.
}

void AudioOff() {
	BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
//	WriteRegister(CS43L22_REG_POWER_CTL1, 0x01);
//	SPI3 ->I2SCFGR = 0;
}

void SetAudioVolume(int volume) {
	BSP_AUDIO_OUT_SetVolume(volume);
//	WriteRegister(CS43L22_REG_MASTER_A_VOL, (volume + 0x19) & 0xff);
//	WriteRegister(CS43L22_REG_MASTER_B_VOL, (volume + 0x19) & 0xff);
}

void OutputAudioSample(int16_t sample) {
	while (!(SPI3 ->SR & SPI_SR_TXE ))
		;
	SPI3 ->DR = sample;
}

void OutputAudioSampleWithoutBlocking(int16_t sample) {
	SPI3 ->DR = sample;
}

void PlayAudioWithCallback(AudioCallbackFunction *callback, void *context) {
	StopAudioDMA();

	NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	NVIC_SetPriority(DMA1_Stream7_IRQn, 0);
//
	SPI3 ->CR2 |= SPI_CR2_TXDMAEN; // Enable I2S TX DMA request.

	CallbackFunction = callback;
	CallbackContext = context;
	BufferNumber = 0;

	if (CallbackFunction)
		CallbackFunction(CallbackContext, BufferNumber);
}

void StopAudio() {

	StopAudioDMA();
	BSP_AUDIO_OUT_Stop(CODEC_PDWN_HW);
//	SPI3 ->CR2 &= ~SPI_CR2_TXDMAEN; // Disable I2S TX DMA request.
//	NVIC_DisableIRQ(DMA1_Stream7_IRQn);
	CallbackFunction = NULL;
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

	if (!DMARunning)
		StartAudioDMAAndRequestBuffers();
	else
		BSP_AUDIO_OUT_ChangeBuffer((uint16_t *) NextBufferSamples, (uint32_t) NextBufferLength);

	NVIC_EnableIRQ(DMA1_Stream7_IRQn);

	return true;
}

//static void WriteRegister(uint8_t address, uint8_t value) {
////	while (I2C1 ->SR2 & I2C_SR2_BUSY )
////		;
////
////	I2C1 ->CR1 |= I2C_CR1_START; // Start the transfer sequence.
////	while (!(I2C1 ->SR1 & I2C_SR1_SB ))
////		; // Wait for start bit.
////
////	I2C1 ->DR = 0x94;
////	while (!(I2C1 ->SR1 & I2C_SR1_ADDR ))
////		; // Wait for master transmitter mode.
////	I2C1 ->SR2;
////
////	I2C1 ->DR = address; // Transmit the address to write to.
////	while (!(I2C1 ->SR1 & I2C_SR1_TXE ))
////		; // Wait for byte to move to shift register.
////
////	I2C1 ->DR = value; // Transmit the value.
////
////	while (!(I2C1 ->SR1 & I2C_SR1_BTF ))
////		; // Wait for all bytes to finish.
////	I2C1 ->CR1 |= I2C_CR1_STOP; // End the transfer sequence.
//	iData[0] = address;
//	iData[1] = value;
//	if(HAL_I2C_Master_Transmit(&hi2c1, AUDIO_I2C_ADDRESS, iData, 2, 100) != HAL_OK)
//	{
//		//TODO: I2C write error
//	}
//}

static void StartAudioDMAAndRequestBuffers() {
	// Configure DMA stream.
//	DMA1_Stream7 ->CR = (0 * DMA_SxCR_CHSEL_0 ) | // Channel 0
//			(1 * DMA_SxCR_PL_0 ) | // Priority 1
//			(1 * DMA_SxCR_PSIZE_0 ) | // PSIZE = 16 bit
//			(1 * DMA_SxCR_MSIZE_0 ) | // MSIZE = 16 bit
//			DMA_SxCR_MINC | // Increase memory address
//			(1 * DMA_SxCR_DIR_0 ) | // Memory to peripheral
//			DMA_SxCR_TCIE; // Transfer complete interrupt
//	DMA1_Stream7 ->NDTR = NextBufferLength;
//	DMA1_Stream7 ->PAR = (uint32_t) &SPI3 ->DR;
//	DMA1_Stream7 ->M0AR = (uint32_t) NextBufferSamples;
//	DMA1_Stream7 ->FCR = DMA_SxFCR_DMDIS;
//	DMA1_Stream7 ->CR |= DMA_SxCR_EN;
	BSP_AUDIO_OUT_Play((uint16_t *) NextBufferSamples, (uint32_t) NextBufferLength);

	// Update state.
	NextBufferSamples = NULL;
	BufferNumber ^= 1;
	DMARunning = true;

	// Invoke callback if it exists to queue up another buffer.
	if (CallbackFunction)
		CallbackFunction(CallbackContext, BufferNumber);
}

static void StopAudioDMA() {
//	DMA1_Stream7 ->CR &= ~DMA_SxCR_EN; // Disable DMA stream.
//	while (DMA1_Stream7 ->CR & DMA_SxCR_EN )
//		; // Wait for DMA stream to stop.
	BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);

	DMARunning = false;
}

void AudioDMA_IRQHandler() {
	DMA1 ->HIFCR |= DMA_HIFCR_CTCIF7; // Clear interrupt flag.

	if (NextBufferSamples) {
		StartAudioDMAAndRequestBuffers();
	} else {
		DMARunning = false;
	}
}