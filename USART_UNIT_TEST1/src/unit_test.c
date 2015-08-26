/**
 * \file
 *
 * \brief SAM USART Unit test
 *
 * Copyright (C) 2013-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage SAM USART Unit Test
 * See \ref appdoc_main "here" for project documentation.
 * \copydetails appdoc_preface
 *
 *
 * \page appdoc_preface Overview
 * This unit test carries out tests for SERCOM USART driver.
 * It consists of test cases for the following functionalities:
 *  - Test for driver initialization.
 *  - Test for single 8-bit write and read by polling.
 *  - Test for single 9-bit write and read by polling.
 *  - Test for multiple 8-bit write by polling and read by interrupts.
 *  - Test for multiple 8-bit write and read by interrupts.
 */

/**
 * \page appdoc_main SAM USART Unit Test
 *
 * Overview:
 * - \ref appdoc_sam0_usart_unit_test_intro
 * - \ref appdoc_sam0_usart_unit_test_setup
 * - \ref appdoc_sam0_usart_unit_test_usage
 * - \ref appdoc_sam0_usart_unit_test_compinfo
 * - \ref appdoc_sam0_usart_unit_test_contactinfo
 *
 * \section appdoc_sam0_usart_unit_test_intro Introduction
 * \copydetails appdoc_preface
 *
 * The following kit is required for carrying out the test:
 *  - SAM D20 Xplained Pro board
 *  - SAM D21 Xplained Pro board
 *  - SAM R21 Xplained Pro board
 *  - SAM L21 Xplained Pro board
 *
 * \section appdoc_sam0_usart_unit_test_setup Setup
 * The following connections has to be made using wires:
 * - SAM D20 Xplained Pro board
 *  - \b TX/RX: EXT1 PIN17 (PA04) <--> EXT1 PIN13 (PB09)
 * - SAM D21 Xplained Pro board
 *  - \b TX/RX: EXT2 PIN17 (PA16) <--> EXT3 PIN17 (PB16)
 * - SAM R21 Xplained Pro board
 *  - \b TX/RX: EXT1 PIN9  (PA22) <--> EXT1 PIN15 (PB03)
 * - SAM L21 Xplained Pro board
 *  - \b TX/RX: EXT2 PIN3  (PA10) <--> EXT2 PIN8  (PB13)
 *
 * To run the test:
 *  - Connect the SAM Xplained Pro board to the computer using a
 *    micro USB cable.
 *  - Open the virtual COM port in a terminal application.
 *    \note The USB composite firmware running on the Embedded Debugger (EDBG)
 *          will enumerate as debugger, virtual COM port and EDBG data
 *          gateway.
 *  - Build the project, program the target and run the application.
 *    The terminal shows the results of the unit test.
 *
 * \section appdoc_sam0_usart_unit_test_usage Usage
 *  - The unit tests are carried out with one SERCOM on EXT1 as the USART
 *    transmitter and another SERCOM on EXT1 as the SERCOM USART receiver.
 *  - Data is transmitted from transmitter to receiver in lengths of a single
 *    byte as well as multiple bytes.
 *
 * \section appdoc_sam0_usart_unit_test_compinfo Compilation Info
 * This software was written for the GNU GCC and IAR for ARM.
 * Other compilers may or may not work.
 *
 * \section appdoc_sam0_usart_unit_test_contactinfo Contact Information
 * For further information, visit
 * <a href="http://www.atmel.com">http://www.atmel.com</a>.
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

#include <asf.h>
#include <stdio_serial.h>
#include <string.h>
#include "conf_test.h"
#include "math.h"
#include "stdio.h"



/* Speed to test USART at */
#define TEST_USART_SPEED   4800

/* Structure for UART module connected to EDBG (used for unit test output) */
struct usart_module cdc_uart_module;

struct usart_module usart_rx_module;
struct usart_config usart_rx_config;
struct usart_module usart_tx_module;
struct usart_config usart_tx_config;

volatile bool transfer_complete;

//#define	ADE7953_LSB		(double)0.000039
#define ACIN						110
#define R37							1000
#define	R38							499000L
#define	R39							499000L

#define VRMS_Full_Scale_Register	9032007L

#define ADC_Full_Scale_VRMS			(double)0.3536
#define P							(double)R37/(R37+R38+R39)
#define INPUT_ATT       			(double)(ACIN*P)
#define	K							(double)(INPUT_ATT/ADC_Full_Scale_VRMS)
#define Ideal_VRM_Register			(double)(VRMS_Full_Scale_Register * K)
#define	ADE7953_LSB					(double)(ACIN/Ideal_VRM_Register)

#define Ideal_GAIN					0x400000L
#define VRMS_Register				0x352256L
#define	IRMSA_Register				0x35423eL
#define	IRMSB_Register				0x35423eL

#define A_AIGAIN					(double)(ADE7953_LSB * IRMSA_Register)
#define	B_AIGAIN					134

#define A_BIGAIN					(double)(ADE7953_LSB * IRMSB_Register)
#define	B_BIGAIN					134

#define A_AVGAIN					(double)(ADE7953_LSB * VRMS_Register)
#define	B_AVGAIN					134


#define Header_Read		0x35
#define Header_Write	0xCA

#define Tx(data)		usart_write_wait(&usart_tx_module, data);
#define Rx(data)		usart_read_wait(&usart_rx_module, (uint16_t*)&data);

#define cmd_Config		0x0102
#define cmd_AENERGYA 	0x021E
#define cmd_IRMSA	 	0x021A
#define cmd_IRMSB	 	0x021B

#define cmd_A			0x0218
#define cmd_VRMS		0x021C
#define cmd_AWATT		0x0212
#define cmd_PFA			0x010A
#define cmd_PFB			0x010B
#define cmd_Period		0x010E
#define cmd_LCYCMODE	0x0004

#define cmd_AIGAIN		0x0280
#define	cmd_AVGAIN		0x0281
#define cmd_BIGAIN		0x028C
#define cmd_BIRMSOS		0x0292
#define cmd_AIRMSOS		0x0286
#define cmd_VRMSOS		0x0288
#define cmd_PFA			0x010A
#define cmd_PFB			0x010B

#define register_8bit	0x01
#define register_16bit	0x02
#define register_24bit	0x03
#define register_32bit	0x04

#define radix_point_size 	3



// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
	int i=0, j=len-1, temp;
	while (i<j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++; j--;
	}
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x)
	{
		str[i++] = (x%10) + '0';
		x = x/10;
	}
	
	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
	str[i++] = '0';
		
	reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
void oem_dtoa(double n, char *res, int afterpoint)
{
	// Extract integer part
	int ipart = (int)n;
	
	// Extract floating part
	double dpart = n - (double)ipart;
		
	// convert integer part to string
	int i = intToStr(ipart, res, 0);
	
	// check for display option after point
	if (afterpoint != 0)
	{
		res[i] = '.';  // add dot
		
		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		dpart = dpart * pow(10, afterpoint);
		
		intToStr((int)dpart, res + i + 1, afterpoint);
	}
}		

/**
 * \internal
 * \brief USART interrupt callback function
 *
 * Called by USART driver when receiving is complete.
 *
 * * \param module USART module causing the interrupt (not used)
 */
static void usart_callback(struct usart_module *const module)
{
	transfer_complete = true;
}

/**
 * \brief Initialize the USART for unit test
 *
 * Initializes the SERCOM USART used for sending the unit test status to the
 * computer via the EDBG CDC gateway.
 */
static void cdc_uart_init(void)
{
	struct usart_config usart_conf;

	/* Configure USART for unit test output */
	usart_get_config_defaults(&usart_conf);
	usart_conf.mux_setting = CONF_STDIO_MUX_SETTING;
	usart_conf.pinmux_pad0 = CONF_STDIO_PINMUX_PAD0;
	usart_conf.pinmux_pad1 = CONF_STDIO_PINMUX_PAD1;
	usart_conf.pinmux_pad2 = CONF_STDIO_PINMUX_PAD2;
	usart_conf.pinmux_pad3 = CONF_STDIO_PINMUX_PAD3;
	usart_conf.baudrate    = CONF_STDIO_BAUDRATE;

	stdio_serial_init(&cdc_uart_module, CONF_STDIO_USART, &usart_conf);
	usart_enable(&cdc_uart_module);
}

static void Write_ADE7953_Register(uint16_t command, uint16_t RegisterSize,uint32_t data)
{
	
	uint16_t timeout_cycles = 0xFFFF;

	/* Send the string */
	//usart_write_buffer_wait(&usart_tx_module, tx_string,3);
	printf("Write data to Register \r\nAddress : 0x%x data : 0x%x\r\n",command,data);

	Tx(Header_Write);
	delay_ms(1);

	Tx((command >> 8));
	delay_ms(1);

	Tx((command & 0x00FF));
	delay_ms(1);
	
	switch(RegisterSize){
		case register_8bit:
			Tx((data & 0x00FF));
			delay_ms(1);		
			break;
		case register_16bit:
			Tx((data & 0x00FF));
			delay_ms(1);
			Tx((data >> 8));
			delay_ms(1);
			break;
		case register_24bit:
			Tx((data & 0x00FF));
			delay_ms(1);
			Tx((data >> 8));
			delay_ms(1);
			Tx((data >> 16));
			delay_ms(1);
			break;
		case register_32bit:
			Tx((data & 0x00FF));
			delay_ms(1);
			Tx((data >> 8));
			delay_ms(1);
			Tx((data >> 16));
			delay_ms(1);
			Tx((data >> 24));
			delay_ms(1);
			break;	
	}
}


static void Read_ADE7953_Register(uint16_t command, uint16_t RegisterSize, uint32_t *Result)
{
	volatile uint8_t rx_string[4] = "";
	uint16_t timeout_cycles = 0xFFFF;

	/* We will read back the buffer using interrupt */
	usart_register_callback(&usart_rx_module, usart_callback,
			USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_rx_module, USART_CALLBACK_BUFFER_RECEIVED);

	/* Start receiving */
	transfer_complete = false;
	usart_read_buffer_job(&usart_rx_module, (uint8_t*)rx_string,RegisterSize);

	/* Send the string */
	//usart_write_buffer_wait(&usart_tx_module, tx_string,3);
	//printf("Read data from Register \r\nAddress : 0x%x data : ",command);
	Tx(Header_Read);
	delay_ms(1);

	Tx((command >> 8));
	delay_ms(1);

	Tx((command & 0x00FF));
	delay_ms(1);
	
	/* Wait until reception completes */
	do {
		timeout_cycles--;
		if (transfer_complete) {
			break;
		}
	} while (timeout_cycles != 0);

	usart_disable_callback(&usart_rx_module, USART_CALLBACK_BUFFER_RECEIVED);
	usart_unregister_callback(&usart_rx_module, USART_CALLBACK_BUFFER_RECEIVED);
	uint32_t temp = 0;
	
	switch(RegisterSize){
			case register_8bit:
				temp = rx_string[0];	
				break;
			case register_16bit:
				temp = (rx_string[1] << 8);
				temp |= rx_string[0]; 
				break;
			case register_24bit:
				temp = (rx_string[2]  << 16);
				temp |= (rx_string[1] << 8);
				temp |= rx_string[0];
				break;
			case register_32bit:
				temp = (rx_string[3]  << 24);
				temp |= (rx_string[2] << 16);
				temp |= (rx_string[1] << 8);
				temp |= rx_string[0];
				break;	
	}
	*Result = temp;
	//printf("0x%x\r\n",temp);
	printf("0x%x  ",temp);
}

static void Read_Write_Test(void)
{
	uint16_t tx_char;
	volatile uint16_t rx_char = 0;

	puts("------------------- Tx ---------------------\r\n");
	/* Write and read the data */
	/*
	tx_char = 0xaa;
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	Tx(tx_char);
	*/
	
	tx_char = (uint16_t)Header_Read;
	Tx(tx_char);
	//printf("tx char = 0x%x\r\n",tx_char);
	//delay_ms(1);

	tx_char = cmd_Config >> 8; 
	Tx(tx_char);
	//printf("tx char = 0x%x\r\n",tx_char);
	//delay_ms(1);
	
	tx_char = cmd_Config & 0x00FF;
	Tx(tx_char);
	//printf("tx char = 0x%x\r\n",tx_char);

	//delay_ms(10);

	//puts("------------------- Rx ---------------------\r\n");
	Rx(rx_char);
	//printf("rx char = 0x%x\r\n",rx_char);

	Rx(rx_char);
	//printf("rx char = 0x%x\r\n",rx_char);

	Rx(rx_char);
	//printf("rx char = 0x%x\r\n",rx_char);
}


/**
 * \brief Initialize USARTs for unit tests
 *
 * Initializes the two USARTs used by the unit test (one for RX and one for TX).
 *
 */
static void test_system_init(void)
{
	/* Configure RX USART */
	usart_get_config_defaults(&usart_rx_config);
	usart_rx_config.mux_setting = CONF_RX_USART_SERCOM_MUX;
	usart_rx_config.pinmux_pad0 = CONF_RX_USART_PINMUX_PAD0;
	usart_rx_config.pinmux_pad1 = CONF_RX_USART_PINMUX_PAD1;
	usart_rx_config.pinmux_pad2 = CONF_RX_USART_PINMUX_PAD2;
	usart_rx_config.pinmux_pad3 = CONF_RX_USART_PINMUX_PAD3;
	usart_rx_config.baudrate    = TEST_USART_SPEED;
	/* Apply configuration */
	usart_init(&usart_rx_module, CONF_RX_USART, &usart_rx_config);
	/* Enable USART */
	usart_enable(&usart_rx_module);

	/* Configure TX USART */
	usart_get_config_defaults(&usart_tx_config);
	usart_tx_config.mux_setting = CONF_TX_USART_SERCOM_MUX;
	usart_tx_config.pinmux_pad0 = CONF_TX_USART_PINMUX_PAD0;
	usart_tx_config.pinmux_pad1 = CONF_TX_USART_PINMUX_PAD1;
	usart_tx_config.pinmux_pad2 = CONF_TX_USART_PINMUX_PAD2;
	usart_tx_config.pinmux_pad3 = CONF_TX_USART_PINMUX_PAD3;
	usart_tx_config.baudrate    = TEST_USART_SPEED;
	/* Apply configuration */
	usart_init(&usart_tx_module, CONF_TX_USART, &usart_tx_config);
	/* Enable USART */
	usart_enable(&usart_tx_module);
}


void Calibration_AI_BI_AV_GAIN(void)
{
	uint32_t 	AIGAIN,BIGAIN,AVGAIN;
	uint32_t	dummy_data;
	double 		Result_Data;

	Read_ADE7953_Register((uint16_t) cmd_IRMSA, (uint16_t) register_24bit,&dummy_data);
	Result_Data = dummy_data * ADE7953_LSB;
	AIGAIN = (B_AIGAIN/Result_Data ) * Ideal_GAIN;	

	Read_ADE7953_Register((uint16_t) cmd_IRMSB, (uint16_t) register_24bit,&dummy_data);
	Result_Data = dummy_data * ADE7953_LSB;
	BIGAIN = (B_BIGAIN/Result_Data ) * Ideal_GAIN;

	Read_ADE7953_Register((uint16_t) cmd_VRMS, (uint16_t) register_24bit,&dummy_data);
	Result_Data = dummy_data * ADE7953_LSB;
	AVGAIN = (B_AVGAIN/Result_Data ) * Ideal_GAIN;

	Write_ADE7953_Register((uint16_t) cmd_AIGAIN, (uint16_t) register_24bit,0x400000);
	Write_ADE7953_Register((uint16_t) cmd_BIGAIN, (uint16_t) register_24bit,0x400000);
	Write_ADE7953_Register((uint16_t) cmd_AVGAIN, (uint16_t) register_24bit,0x400000);

	Read_ADE7953_Register((uint16_t) cmd_AIGAIN, (uint16_t) register_24bit,&dummy_data);
	Read_ADE7953_Register((uint16_t) cmd_BIGAIN, (uint16_t) register_24bit,&dummy_data);
	Read_ADE7953_Register((uint16_t) cmd_AVGAIN, (uint16_t) register_24bit,&dummy_data);

	delay_ms(100);

	Write_ADE7953_Register((uint16_t) cmd_AIGAIN, (uint16_t) register_24bit,AIGAIN);
	Write_ADE7953_Register((uint16_t) cmd_BIGAIN, (uint16_t) register_24bit,BIGAIN);
	Write_ADE7953_Register((uint16_t) cmd_AVGAIN, (uint16_t) register_24bit,AVGAIN);
	
	Read_ADE7953_Register((uint16_t) cmd_AIGAIN, (uint16_t) register_24bit,&dummy_data);
	Read_ADE7953_Register((uint16_t) cmd_BIGAIN, (uint16_t) register_24bit,&dummy_data);
	Read_ADE7953_Register((uint16_t) cmd_AVGAIN, (uint16_t) register_24bit,&dummy_data);
	printf("\r\n");

}
/**
 * \brief Run USART unit tests
 *
 * Initializes the system and serial output, then sets up the
 * USART unit test suite and runs it.
 */
int main(void)
{
	uint32_t	count;
	char 		myString[8];
	double 		Result_Data;
	double 		I_data;
	double		V_data,P_data;
	uint32_t	dummy_data;
	
	system_init();
	cdc_uart_init();
	test_system_init();
	Calibration_AI_BI_AV_GAIN();
	//ADE7953_Init();
	
	//double data;
	//uint32_t AIGAIN;
	//uint32_t BIGAIN;
	//uint32_t AVGAIN;
		
	//AIGAIN = (B_AIGAIN/A_AIGAIN) * Ideal_GAIN;
	//BIGAIN = (B_BIGAIN/A_BIGAIN) * Ideal_GAIN;
	//AVGAIN = (B_AVGAIN/A_AVGAIN) * Ideal_GAIN;
	
	//Read_ADE7953_Register((uint16_t) cmd_IRMSA, (uint16_t) register_24bit,&dummy_data);
	//Result_Data = dummy_data * ADE7953_LSB;
	//AIGAIN = (B_AIGAIN/Result_Data ) * Ideal_GAIN;	

	//Read_ADE7953_Register((uint16_t) cmd_IRMSB, (uint16_t) register_24bit,&dummy_data);
	//Result_Data = dummy_data * ADE7953_LSB;
	//BIGAIN = (B_BIGAIN/Result_Data ) * Ideal_GAIN;

	//Read_ADE7953_Register((uint16_t) cmd_VRMS, (uint16_t) register_24bit,&dummy_data);
	//Result_Data = dummy_data * ADE7953_LSB;
	//AVGAIN = (B_AVGAIN/Result_Data ) * Ideal_GAIN;
	
	count = 1;

	//Write_ADE7953_Register((uint16_t) cmd_LCYCMODE, (uint16_t) register_8bit,(uint32_t)0x01);
	//printf("\r\n");
	
	//Write_ADE7953_Register((uint16_t) cmd_AIGAIN, (uint16_t) register_24bit,0x400000);
	//Write_ADE7953_Register((uint16_t) cmd_BIGAIN, (uint16_t) register_24bit,0x400000);
	//Write_ADE7953_Register((uint16_t) cmd_AVGAIN, (uint16_t) register_24bit,0x400000);

	//Read_ADE7953_Register((uint16_t) cmd_AIGAIN, (uint16_t) register_24bit,&dummy_data);
	//Read_ADE7953_Register((uint16_t) cmd_BIGAIN, (uint16_t) register_24bit,&dummy_data);
	//Read_ADE7953_Register((uint16_t) cmd_AVGAIN, (uint16_t) register_24bit,&dummy_data);

	//Write_ADE7953_Register((uint16_t) cmd_AIGAIN, (uint16_t) register_24bit,AIGAIN);
	//Write_ADE7953_Register((uint16_t) cmd_BIGAIN, (uint16_t) register_24bit,BIGAIN);
	//Write_ADE7953_Register((uint16_t) cmd_AVGAIN, (uint16_t) register_24bit,AVGAIN);
	
	//Read_ADE7953_Register((uint16_t) cmd_AIGAIN, (uint16_t) register_24bit,&dummy_data);
	//Read_ADE7953_Register((uint16_t) cmd_BIGAIN, (uint16_t) register_24bit,&dummy_data);
	//Read_ADE7953_Register((uint16_t) cmd_AVGAIN, (uint16_t) register_24bit,&dummy_data);
	

	while (true) {
		printf("# %d\r\n",count++);

		//Write_ADE7953_Register((uint16_t) cmd_LCYCMODE, (uint16_t) register_8bit,(uint32_t)0x81);
		//printf("\r\n");

		//Read_ADE7953_Register((uint16_t) cmd_LCYCMODE, (uint16_t) register_8bit,&dummy_data);
		//printf("\r\n\r\n");
		
		//printf("\r\nAccess Config Register #%d\r\n",count++);
		
		//Write_ADE7953_Register((uint16_t) cmd_Config, (uint16_t) register_16bit,(uint32_t)0x8007);

		//Read_ADE7953_Register((uint16_t) cmd_Config, (uint16_t) register_16bit,&dummy_data);

		//puts("\r\nSoftware Reset\r\n");

		//Write_ADE7953_Register((uint16_t) cmd_Config, (uint16_t) register_16bit,(uint32_t)(0x8007|0x0080));
		
		//Read_ADE7953_Register((uint16_t) cmd_Config, (uint16_t) register_16bit);

		//printf("\r\n");
		
		//Read_ADE7953_Register((uint16_t) cmd_Config, (uint16_t) register_16bit);

		printf("IRMSA ");
		Read_ADE7953_Register((uint16_t) cmd_IRMSA, (uint16_t) register_24bit,&dummy_data);
		Result_Data = dummy_data * ADE7953_LSB;
		I_data = Result_Data; 
		oem_dtoa(Result_Data,myString,radix_point_size);
		printf(myString);printf("\r\n\r\n");

		//printf("IRMSB ");
		//Read_ADE7953_Register((uint16_t) cmd_IRMSB, (uint16_t) register_24bit,&dummy_data);
		//Result_Data = dummy_data * ADE7953_LSB;
		//oem_dtoa(Result_Data,myString,radix_point_size);
		//printf(myString);printf("\r\n\r\n");
		
		printf("VRMS ");
		Read_ADE7953_Register((uint16_t) cmd_VRMS, (uint16_t) register_24bit,&dummy_data);
		Result_Data = dummy_data * ADE7953_LSB;
		V_data = Result_Data; 
		oem_dtoa(Result_Data,myString,radix_point_size);
		printf(myString);printf("\r\n\r\n");

		printf("Period ");
		Read_ADE7953_Register((uint16_t) cmd_Period, (uint16_t) register_16bit,&dummy_data);
		printf("%d\r\n\r\n",dummy_data);
		
		//printf("AENERGYA ");
		//Read_ADE7953_Register((uint16_t) cmd_AENERGYA, (uint16_t) register_24bit,&dummy_data);

		printf("AWATT ");
		Read_ADE7953_Register((uint16_t) cmd_AWATT, (uint16_t) register_24bit,&dummy_data);
		printf(" %d\r\n\r\n",dummy_data);

		//Result_Data = dummy_data * 0.000039;
		//oem_dtoa(Result_Data,myString,5);
		//printf(myString);printf("\r\n");

		P_data = V_data * I_data;		
		oem_dtoa(P_data,myString,radix_point_size);
		printf("Power : ");
		printf(myString);printf("\r\n\r\n");

		//printf("AIRMSOS ");
		//Read_ADE7953_Register((uint16_t) cmd_AIRMSOS, (uint16_t) register_24bit,&dummy_data);
		

		//printf("BIRMSOS ");
		//Read_ADE7953_Register((uint16_t) cmd_BIRMSOS, (uint16_t) register_24bit,&dummy_data);
		

		//printf("VRMSOS ");
		//Read_ADE7953_Register((uint16_t) cmd_VRMSOS, (uint16_t) register_24bit,&dummy_data);

		printf("PFA ");
		Read_ADE7953_Register((uint16_t) cmd_PFA, (uint16_t) register_16bit,&dummy_data);
		printf("\r\n");

		//printf("PFB ");
		//Read_ADE7953_Register((uint16_t) cmd_PFB, (uint16_t) register_16bit,&dummy_data);
		//printf("\r\n");
		
		delay_ms(1000);
	}

}
