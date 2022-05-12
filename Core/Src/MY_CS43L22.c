/*
Library:					STM32F4 Audio Codec - CS43L22
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			29/01/2016
Last modified:			29/12/2018
Description:			This is an STM32 device driver library for the CS43L22 Audio Codec, using STM HAL libraries

References:
			1) Cirrus Logic CS43L22 datasheet
				 https://www.mouser.com/ds/2/76/CS43L22_F2-1142121.pdf
			2) ST opensource CS43L22 Audio Codec dsp drivers.
										
* Copyright (C) 2018 - M. Yaqoob
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.
	
   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

#include "stm32f4xx_hal.h"
#include "MY_CS43L22.h"

static uint8_t iData[4];
I2C_HandleTypeDef* i2cx;
extern I2S_HandleTypeDef hi2s3;
extern DMA_HandleTypeDef hdma_spi3_tx;
uint8_t id;
uint8_t rev;
uint8_t vp, spk_status, status;
//(1): Functions definitions
//-------------- Static Functions ---------------//
// Function(1): Write to register
static void write_register(uint8_t reg, uint8_t *data)
{
//	iData[0] = reg;
//	iData[1] = data[0];
	HAL_I2C_Mem_Write(i2cx, DAC_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1,1000);
//	HAL_I2C_Master_Transmit(i2cx, DAC_I2C_ADDR, iData, 2, 100);
//	HAL_I2C_Master_Transmit(i2cx, DAC_I2C_ADDR, data, 1, 100);
}
// Function(2): Read from register
static void read_register(uint8_t reg, uint8_t *data)
{
//	iData[0] = reg;
	HAL_I2C_Mem_Read(i2cx, DAC_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 1000);
//	HAL_I2C_Master_Transmit(i2cx, DAC_I2C_ADDR, iData, 1, 100);
//	HAL_I2C_Master_Receive(i2cx, DAC_I2C_ADDR, data, 1, 100);
}

//-------------- Public Functions ----------------//
// Function(1): Initialisation
void CS43_Init(I2C_HandleTypeDef* i2c_handle, CS43_MODE outputMode)
{
//	__HAL_UNLOCK(&hi2s3);     // THIS IS EXTREMELY IMPORTANT FOR I2S3 TO WORK!!
//	__HAL_I2S_ENABLE(&hi2s3); // THIS IS EXTREMELY IMPORTANT FOR I2S3 TO WORK!!
	//(1): Get the I2C handle
	i2cx = i2c_handle;

	if (i2cx == NULL) {
		Error_Handler();
	}

	HAL_I2C_Init(i2cx);
	HAL_I2S_Init(&hi2s3);
	HAL_DMA_Init(&hdma_spi3_tx);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

	for (volatile int i = 0; i < 0x4fff; i++) {
		__asm__ volatile("nop");
	}

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);

	//(2): Power down
	iData[1] = 0x01;
	write_register(POWER_CONTROL1,&iData[1]);

	//(2-1): Get Chip ID
	read_register(CHIP_ID, &iData[1]);
	id = iData[1] >> 3; //default 11100 (0x1D)
	rev = iData[1] & 0x07; // 000 (0:A0), 001 (1:A1), 010 (2:B0), 011 (3:B1)

	//(3): Enable Right and Left headphones
	iData[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
	iData[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
	iData[1] |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	iData[1] |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	write_register(POWER_CONTROL2,&iData[1]);

	//(4): Automatic clock detection
	iData[1] = (1 << 7);
	write_register(CLOCKING_CONTROL,&iData[1]);

	//(5): Interface control 1
	read_register(INTERFACE_CONTROL1, &iData[1]);
	iData[1] &= ~(1 << 5); // Clear all bits except bit 5 which is reserved
	iData[1] &= ~(1 << 7);  // Slave
	iData[1] &= ~(1 << 6);  // Clock polarity: Not inverted
	iData[1] &= ~(1 << 4);  // No DSP mode
//	iData[1] &= ~(1 << 2);  // Left justified, up to 24 bit (default)
	iData[1] |= (1 << 2);	// I2S up to 24 bit
	
	iData[1] |=  (3 << 0);  // 16-bit audio word length for I2S interface
	write_register(INTERFACE_CONTROL1,&iData[1]);

	//(6): Passthrough A settings
	read_register(PASSTHROUGH_A, &iData[1]);
	iData[1] &= 0xF0;      // Bits [4-7] are reserved
	iData[1] |=  (1 << 0); // Use AIN1A as source for passthrough
//	iData[1] |=  (1 << 3); // Use AIN4A as source for passthrough
	write_register(PASSTHROUGH_A,&iData[1]);

	//(7): Passthrough B settings
	read_register(PASSTHROUGH_B, &iData[1]);
	iData[1] &= 0xF0;      // Bits [4-7] are reserved
	iData[1] |=  (1 << 0); // Use AIN1B as source for passthrough
//	iData[1] |=  (1 << 3); // Use AIN4B as source for passthrough
	write_register(PASSTHROUGH_B,&iData[1]);

	//(8): Miscellaneous register settings
	read_register(MISCELLANEOUS_CONTRLS, &iData[1]);
	if(outputMode == MODE_AN)
	{
		iData[1] |=  (1 << 7);   // Enable passthrough for AIN-A
		iData[1] |=  (1 << 6);   // Enable passthrough for AIN-B
		iData[1] &= ~(1 << 5);   // Unmute passthrough on AIN-A
		iData[1] &= ~(1 << 4);   // Unmute passthrough on AIN-B
		iData[1] &= ~(1 << 3);   // Changed settings take affect immediately
	}
	else if(outputMode == MODE_I2S)
	{
		iData[1] = 0x02;
	}
	write_register(MISCELLANEOUS_CONTRLS,&iData[1]);

	//(9-1): Gain headphone and speaker
	read_register(PLAYBACK_CONTROL_1, &iData[1]);
	iData[1] |= (3 << 5);	//Headphone Analog Gain 011 (default)
	iData[1] &= ~(1 << 4);	//Playback Volume Setting B=A
	iData[1] &= ~(3 << 0);	//Master Playback Mute OFF
	write_register(PLAYBACK_CONTROL_1,&iData[1]);

	//(9-2): Unmute headphone and speaker
	read_register(PLAYBACK_CONTROL_2, &iData[1]);
	iData[1] &= ~(3 << 6);	//HP Mute disable
	iData[1] |= (3 << 4);	//SP Mute enable
	iData[1] |= (1 << 3);	//SP A=B
	iData[1] &= ~(1 << 2);	//Not swap channel
	iData[1] |= (1 << 1);	//Mono
	iData[1] &= ~(1 << 0);	//Disable mute 50/50
	write_register(PLAYBACK_CONTROL_2,&iData[1]);

	//(10): Set volume to default (0dB)
	iData[1] = 0x00;
	write_register(PASSTHROUGH_VOLUME_A,&iData[1]);
	write_register(PASSTHROUGH_VOLUME_B,&iData[1]);
	write_register(PCM_VOLUME_A,&iData[1]);
	write_register(PCM_VOLUME_B,&iData[1]);

	iData[1] = 0x0B;
	iData[1] |= (0x01 << 7); //Battery Compensation enable
	iData[1] |= (0x01 << 6); //VP Monitor
	write_register(CS43L22_REG_BAT, &iData[1]);
}

// Function(2): Enable Right and Left headphones
void CS43_Enable_RightLeft(uint8_t side)
{
	switch (side)
	{
		case 0:
			iData[1] =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		case 1:
			iData[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (3 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		case 2:
			iData[1] =  (3 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		case 3:
			iData[1] =  (2 << 6);  // PDN_HPB[0:1]  = 10 (HP-B always onCon)
			iData[1] |= (2 << 4);  // PDN_HPA[0:1]  = 10 (HP-A always on)
			break;
		default:
			break;
	}
	iData[1] |= (3 << 2);  // PDN_SPKB[0:1] = 11 (Speaker B always off)
	iData[1] |= (3 << 0);  // PDN_SPKA[0:1] = 11 (Speaker A always off)
	write_register(POWER_CONTROL2,&iData[1]);
}

// Function(3): Set Volume Level
void CS43_SetVolume(uint8_t volume)
{
	int8_t tempVol = volume - 50;
	tempVol = tempVol*(127/50);
	uint8_t myVolume =  (uint8_t )tempVol;
	iData[1] = myVolume;
	write_register(PASSTHROUGH_VOLUME_A,&iData[1]);
	write_register(PASSTHROUGH_VOLUME_B,&iData[1]);
	
	iData[1] = VOLUME_CONVERT_D(volume);
	
	/* Set the Master volume */ 
	write_register(CS43L22_REG_MASTER_A_VOL,&iData[1]);
	write_register(CS43L22_REG_MASTER_B_VOL,&iData[1]);
}

// Function(4): Start the Audio DAC
void CS43_Start(void)
{
	// Write 0x99 to register 0x00.
	iData[1] = 0x99;
	write_register(CONFIG_00,&iData[1]);
	// Write 0x80 to register 0x47.
	iData[1] = 0x80;
	write_register(CONFIG_47,&iData[1]);
	// Write '1'b to bit 7 in register 0x32.
	read_register(CONFIG_32, &iData[1]);
	iData[1] |= 0x80;
	write_register(CONFIG_32,&iData[1]);
	// Write '0'b to bit 7 in register 0x32.
	read_register(CONFIG_32, &iData[1]);
	iData[1] &= ~(0x80);
	write_register(CONFIG_32,&iData[1]);
	// Write 0x00 to register 0x00.
	iData[1] = 0x00;
	write_register(CONFIG_00,&iData[1]);

	//Set the "Power Ctl 1" register (0x02) to 0x9E
	iData[1] = 0x9E;
	write_register(POWER_CONTROL1,&iData[1]);
}

void CS43_Stop(void)
{
	iData[1] = 0x01;
	write_register(POWER_CONTROL1,&iData[1]);
}

void CS43_Beep_ON(void)
{
	iData[1] = 0x00;
	iData[1] |= (0x0E << 4); //Frequency 2KHz
	iData[1] |= (0x0F << 0); //duration ~5.2s
	write_register(BEEP_FREQ,&iData[1]);

	iData[1] = 0x00;
	iData[1] |= (0x01 << 5); //Offtime ~2.58s
	iData[1] |= (6 << 0); //vol gain 6dB
	write_register(BEEP_VOLUME,&iData[1]);

	iData[1] = 0x00;
	iData[1] |= (0x02 << 6); //00:off 01single 10:multiple 11:continuous mix
	iData[1] &= ~(1 << 5); //Mix enable
	write_register(BEEP_TONE,&iData[1]);
}

void CS43_Beep_OFF(void)
{
	iData[1] = 0x00;
//	iData[1] |= (0x03 << 6); //continuous mix
//	iData[1] &= ~(1 << 5); //Mix enable
	write_register(BEEP_TONE, &iData[1]);
}

void CS43_Status(cs43_status_t* t)
{
	read_register(CS43L22_REG_STATUS, &t->status);

	read_register(CS43L22_REG_VP_MON, &t->vp);

	read_register(CS43L22_REG_SP_MON, &t->spk_status);

	return ;
}
