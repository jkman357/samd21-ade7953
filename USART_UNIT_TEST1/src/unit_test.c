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
#define ACIN						(double)121.05
#define R37							1000
#define	R38							499000L
#define	R39							499000L

#define VRMS_Full_Scale_Register	9032007L

#define ADC_Full_Scale_VRMS			(double)0.3535533
#define P							(double)R37/(R37+R38+R39)
#define INPUT_ATT       			(double)(ACIN*P)
#define	K							(double)(INPUT_ATT/ADC_Full_Scale_VRMS)
#define Ideal_VRMS_Register			(double)(VRMS_Full_Scale_Register * K)
#define	ADE7953_VRMS_LSB			(double)(ACIN/Ideal_VRMS_Register)

#define IRMS_Full_Scale_Register	9032007L
#define ADC_Full_Scale_IRMS			(double)0.3535533
#define I_INPUT						(double)1.397
#define R_Shunt						(double)0.003
#define	IPGA						1
#define I_OUTPUT					(double)(I_INPUT * R_Shunt * IPGA)
#define I_Scale						(double)(I_OUTPUT/ADC_Full_Scale_IRMS)
#define Ideal_IRMS_Register			(double)(IRMS_Full_Scale_Register * I_Scale)
#define ADE7953_IRMS_LSB			(double)(I_INPUT/Ideal_IRMS_Register)

#define Ideal_GAIN					0x400000L
#define	B_AIGAIN					110
#define	B_BIGAIN					110
#define	B_AVGAIN					110

#define AWATT_Full_Register_LSB		(double)((ADE7953_VRMS_LSB * VRMS_Full_Scale_Register) * 	\
											(ADE7953_IRMS_LSB * IRMS_Full_Scale_Register))/ 	\
											4862401L

#define Header_Read		0x35
#define Header_Write	0xCA

#define Tx(data)		usart_write_wait(&usart_tx_module, data);
#define Rx(data)		usart_read_wait(&usart_rx_module, (uint16_t*)&data);

#define SAGCYC 			0x000  		//R/W 8 0x00 U Sag Line Cycle Register
#define LCYCMODE		0x004  		//R/W 8 0x40 U Line cycle Accumulation Mode Configuration
#define PGA_V			0x007  		//R/W 8 0x00 U Voltage Channel Gain Configuration
#define PGA_IA			0x008  		//R/W 8 0x00 U Current Channel A Gain Configuration
#define	PGA_IB			0x009  		//R/W 8 0x00 U Current Channel B Gain Configuration
#define	Write_protect	0x040  		//R/W 8 0x00 U Write protection bits [2:0]
#define	Last_Op			0xFD  		//R 8 0x00 U Contains the
#define	Last_rwdata8	0xFF  		//R 8 0x00 U Contains the data from the last successful 8 bit register communication
#define	EX_REF			0x800  		//R/W 8 0x00 U Reference input configuration. 0 for internal, set to 1 for external


#define	ZXTOUT			0x100  		//W/R 16 0xFFFF U Zero Crossing Timeout
#define	LINECYC			0x101  		//R/W 16 0x00 U Line Cycle Energy Accumulation Mode Line-Cycle Register
#define	CONFIG			0x102  		//W/R 16 0x8004 U Configuration Register
#define	CF1DEN			0x103  		//R/W 16 0x3F U CF1 Frequency Divider Denominator Register
#define	CF2DEN			0x104  		//R/W 16 0x3F U CF2 Frequency Divider Denominator Register
#define	CFMODE			0x107  		//R/W 16 0x300 U CF output Selection
#define	PHCALA			0x108  		//R/W 16 0x00 S Phase Calibration Register (channel A)
#define	PHCALB			0x109  		//R/W 16 0x00 S Phase Calibration Register (channel B)
#define	PFA				0x10A  		//R 16 0x00 S Power Factor channel A
#define	PFB				0x10B  		//R 16 0x00 S Power Factor Channel B
#define	Angle_A			0x10C  		//R 16 0x00 S Angle between voltage and Current A
#define	Angle_B			0x10D  		//R 16 0x00 S Angle between voltage and Current B
#define	PERIOD			0x10E  		//R 16 0x00 U Period Register
#define	ALT_Output		0x110  		//R/W 16 0x00 U Alternative Output Functions
#define	Last_Add		0x1FE  		//R 16 0x00 U Contains the address of the last successful communication
#define	Last_rwdata16	0x1FF  		//R 16 0x00 U Contains the data from the last successive 16 bit register communication

#define SAGLVL			0x300  		//R/W 24/32 0x00 U SAG Voltage Level
#define ACCMODE			0x301  		//R/W 24/32 0x00 U Accumulation Mode
#define AP_NOLOAD		0x303  		//R/W 24/32 0x00 U Active Power No Load Level
#define VAR_NOLOAD		0x304  		//R/W 24/32 0xE419 U Reactive Power No Load Level
#define VA_NLOAD		0x305  		//R/W 24/32 0xE419 U Apparent Power No Load Level
#define AVA				0x310  		//R 24/32 0x00 S Instantaneous Apparent Power A
#define BVA				0x311  		//R 24/32 0x00 S Instantaneous Apparent Power B
#define AWATT			0x312  		//R 24/32 0x00 S Instantaneous Active Power A
#define BWATT			0x313  		//R 24/32 0x00 S Instantaneous Active Power B
#define AVAR			0x314  		//R 24/32 0x00 S Instantaneous Reactive Power A
#define BVAR			0x315  		//R 24/32 0x00 S Instantaneous Reactive Power B
#define IA				0x316  		//R 24/32 0x00 S Instantaneous Current Channel A
#define IB				0x317  		//R 24/32 0x00 S Instantaneous Current Channel B
#define VA				0x318  		//R 24/32 0x00 S Instantaneous Voltage Channel A
#define VB				0x319  		//R 24/32 0x00 S Instantaneous Voltage Channel B
#define IRMSA			0x31A  		//R 24/32 0x00 U IRMS Register A (channel A)
#define IRMSB			0x31B  		//R 24/32 0x00 U IRMS Register B (channel B)
#define VRMS			0x31C  		//R 24/32 0x00 U VRMS Register
#define AENERGYA		0x31E  		//R 24/32 0x00 S Active Energy Register (channel A)
#define AENERGYB		0x31F  		//R 24/32 0x00 S Active Energy Register (channel B)
#define RENERGYA		0x320  		//R 24/32 0x00 S Reactive Energy Register (channel A)
#define RENERGYB		0x321  		//R 24/32 0x00 S Reactive Energy Register (channel B)
#define APENERGYA		0x322  		//R 24/32 0x00 S Apparent Energy Register (channel A)
#define APENERGYB		0x323  		//R 24/32 0x00 S Apparent Energy Register (channel B)
#define OVLVL			0x324  		//R/W 24/32 0xFFFFFF U Over Voltage Level
#define OILVL			0x325  		//R/W 24/32 0xFFFFFF U Over Current Level
#define VPEAK			0x326  		//R 24/32 0x00 U Voltage Channel Peak Register
#define RSTVPEAK		0x327  		//R 24/32 0x00 U Read Voltage Peak with Reset
#define IAPEAK			0x328  		//R 24/32 0x00 U Current Channel A Peak Register
#define RSTIAPEAK		0x329  		//R 24/32 0x00 U Read Current Channel A Peak with Reset
#define IBPEAK			0x32A  		//R 24/32 0x00 U Current Channel B Peak Register
#define RSTIBPEAK		0x32B  		//R 24/32 0x00 U Read Current Channel B Peak with Reset
#define IRQENA			0x32C  		//R/W 24/32 0x100000 U Interrupt Enable Register
#define IRQSTATA		0x32D  		//R 24/32 0x00 U Interrupt Status Register
#define RSTIRQSTATA		0x32E  		//R 24/32 0x00 U Reset Interrupt Status register
#define IRQENB			0x32F  		//R/W 24/32 0x00 U Interrupt B Enable Register
#define IRQSTATB		0x330  		//R 24/32 0x00 U Interrupt B Status Register
#define RSTIRQSTATB		0x331  		//R 24/32 0x00 U Reset Interrupt B Status register
#define CRC				0x37F  		//R 32 0xC02F1AD4 U Check Sum
#define AIGAIN			0x380  		//R/W 24/32 0x400000 U Current Channel Gain (channel A) 每 23 bit
#define AVGAIN			0x381  		//R/W 24/32 0x400000 U Voltage Channel Gain 每 23 bit
#define AWGAIN			0x382  		//R/W 24/32 0x400000 U Active Power Gain (channel A) 每 23 bit
#define AVARGAIN		0x383  		//R/W 24/32 0x400000 U Reactive Power Gain (channel A) 每 23 bit
#define AVAGAIN			0x384  		//R/W 24/32 0x400000 U Apparent Power Gain (channel A) 每 23 bit
//#define AIOS 			0x385 		//R/W 24/32 0x00 S Current Channel Offset (channel A)
#define AIRMSOS			0x386  		//R/W 24/32 0x00 S IRMS Offset (channel A)
//#define AVOS			0x387  		//R/W 24/32 0x00 S Voltage Channel Offset
#define AVRMSOS			0x388  		//R/W 24/32 0x00 S VRMS Offset
#define AWATTOS			0x389  		//R/W 24/32 0x00 S Active Power Offset Correction (channel A)
#define AVAROS			0x38A  		//R/W 24/32 0x00 S Reactive Power Offset Correction (channel A)
#define AVAOS			0x38B  		//R/W 24/32 0x00 S Apparent Power Offset Correction (channel A)
#define BIGAIN			0x38C  		//R/W 24/32 0x400000 S Current Channel Gain (channel B)
#define BVGAIN			0x38D  		//R/W 24/32 0x400000 S Voltage Channel Gain
#define BWGAIN			0x38E  		//R/W 24/32 0x400000 S Active Power Gain (channel B)
#define BVARGAIN		0x38F  		//R/W 24/32 0x400000 S Reactive Power Gain (channel B)
#define BVAGAIN			0x390  		//R/W 24/32 0x400000 S Apparent Power Gain (channel B)
#define BIOS			0x391  		//R/W 24/32 0x00 S Current Channel Offset (channel B)
#define BIRMSOS			0x392  		//R/W 24/32 0x00 S IRMS Offset (channel B)
#define BVOS			0x393  		//R/W 24/32 0x00 S Voltage Channel Offset
#define BVRMSOS			0x394  		//R/W 24/32 0x00 S VRMS Offset
#define BWATTOS			0x395  		//R/W 24/32 0x00 S Active Power Offset Correction (channel B)
#define BVAROS			0x396  		//R/W 24/32 0x00 S Reactive Power Offset Correction (channel B)
#define BVAOS			0x397  		//R/W 24/32 0x00 S Apparent Power Offset Correction (channel B)
#define Last_rwdata32	0x3FF  		//R 24/32 0x00 U Contains the data from the last successive 24/32 bit register communication
#define VERSION         0x702

#define   CFDEN6400    	125       	
#define   AIGAIN_C		0
#define   AVGAIN_C		0
#define   AIRMSOS_C		0
#define   AVRMSOS_C		0
#define   BIGAIN_C		0
#define   BVGAIN_C		0
#define   BIRMSOS_C		0
#define   BVRMSOS_C		0
#define   CIGAIN_C		0
#define   CVGAIN_C		0
#define   CIRMSOS_C		0
#define   CVRMSOS_C		0
#define   APHCAL_C		0
#define   BPHCAL_C		0
#define   CPHCAL_C		0
/* 	#define   VANOLOAD_C	2503
 	#define   APNOLOAD_C	2503
 	#define   VARNOLOAD_C	2503	*/
#define   VANOLOAD_C	25
#define   APNOLOAD_C	25
#define   VARNOLOAD_C	25
#define   VATHR1_C		0x01
#define   VATHR0_C		0xF3E709
#define   WTHR1_C		0x01
#define   WTHR0_C		0xF3E709
#define   VARTHR1_C		0x01
#define   VARTHR0_C		0xF3E709
#define   VLEVEL_C		402885


#define   EXTERREFEN            0x01
#define   INTERREFEN            0xFE
#define   PORT_LOCK             0x02
#define   WATT2CF             	0xFFF8
#define   VAR2CF             	0xFFF9
#define   FWATT2CF             	0xFFFB
#define   CF1EN             	0xFDFF
#define   CF2EN             	0xFBFF
#define   CF3EN             	0xF7FF
#define   Angle_Phase           0xF3FF
#define   Angle_Voltage         0xF7FF
#define   Angle_Current         0xFBFF
#define   Period_A           	0xFC
#define   Period_B           	0xFD
#define   Period_C           	0xFE

#define   PF_LSB					(double)0.000030517578125
//#define cmd_Config		0x0102
//#define cmd_AENERGYA 		0x021E
//#define cmd_IRMSA	 		0x021A
//#define cmd_IRMSB	 		0x021B

//#define cmd_A				0x0218
//#define cmd_VRMS			0x021C
//#define cmd_AWATT			0x0212
//#define cmd_PFA			0x010A
//#define cmd_PFB			0x010B
//#define cmd_Period		0x010E
//#define cmd_LCYCMODE		0x0004

//#define cmd_AIGAIN		0x0280
//#define cmd_AVGAIN		0x0281
//#define cmd_BIGAIN		0x028C
//#define cmd_BIRMSOS		0x0292
//#define cmd_AIRMSOS		0x0286
//#define cmd_VRMSOS		0x0288
//#define cmd_PFA			0x010A
//#define cmd_PFB			0x010B

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

	tx_char = CONFIG >> 8; 
	Tx(tx_char);
	//printf("tx char = 0x%x\r\n",tx_char);
	//delay_ms(1);
	
	tx_char = CONFIG & 0x00FF;
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
	uint32_t 	AIgain,BIgain,AVgain;
	uint32_t	dummy_data;
	double 		Result_Data;

	Write_ADE7953_Register((uint16_t) AIGAIN, (uint16_t) register_32bit,0x400000);
	Write_ADE7953_Register((uint16_t) BIGAIN, (uint16_t) register_32bit,0x400000);
	Write_ADE7953_Register((uint16_t) AVGAIN, (uint16_t) register_32bit,0x400000);

	Read_ADE7953_Register((uint16_t) AIGAIN, (uint16_t) register_32bit,&dummy_data);
	Read_ADE7953_Register((uint16_t) BIGAIN, (uint16_t) register_32bit,&dummy_data);
	Read_ADE7953_Register((uint16_t) AVGAIN, (uint16_t) register_32bit,&dummy_data);
	printf("\r\n");
	
	Read_ADE7953_Register((uint16_t) IRMSA, (uint16_t) register_32bit,&dummy_data);
	AIgain = Ideal_GAIN/ (dummy_data/Ideal_IRMS_Register);	 
		
	Read_ADE7953_Register((uint16_t) IRMSB, (uint16_t) register_32bit,&dummy_data);
	BIgain = Ideal_GAIN/ (dummy_data/Ideal_IRMS_Register);
	
	Read_ADE7953_Register((uint16_t) VRMS, (uint16_t) register_32bit,&dummy_data);
	AVgain = Ideal_GAIN/(dummy_data/Ideal_VRMS_Register);

	printf("\r\n");
	Write_ADE7953_Register((uint16_t) AIGAIN, (uint16_t) register_32bit,AIgain);
	Write_ADE7953_Register((uint16_t) BIGAIN, (uint16_t) register_32bit,BIgain);
	Write_ADE7953_Register((uint16_t) AVGAIN, (uint16_t) register_32bit,AVgain);
	
	Read_ADE7953_Register((uint16_t) AIGAIN, (uint16_t) register_32bit,&dummy_data);
	Read_ADE7953_Register((uint16_t) BIGAIN, (uint16_t) register_32bit,&dummy_data);
	Read_ADE7953_Register((uint16_t) AVGAIN, (uint16_t) register_32bit,&dummy_data);
	printf("\r\n");

	//while(1);
	delay_s(5);
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
	double		V_data,P_data,Freq,Dtime,temp_lsb,kwatt;
	uint32_t	dummy_data;
	
	system_init();
	cdc_uart_init();
	test_system_init();
	Calibration_AI_BI_AV_GAIN();
		
	count = 1;	
	kwatt = 0;
	
	while (true) {
		printf("# %d\r\n",count++);

		printf("IRMSA ");
		Read_ADE7953_Register((uint16_t) IRMSA, (uint16_t) register_32bit,&dummy_data);
		Result_Data = dummy_data * ADE7953_IRMS_LSB;
		I_data = Result_Data; 
		oem_dtoa(Result_Data,myString,radix_point_size);
		printf(myString);printf("\r\n\r\n");

		//printf("IRMSB ");
		//Read_ADE7953_Register((uint16_t) cmd_IRMSB, (uint16_t) register_24bit,&dummy_data);
		//Result_Data = dummy_data * ADE7953_LSB;
		//oem_dtoa(Result_Data,myString,radix_point_size);
		//printf(myString);printf("\r\n\r\n");
		
		printf("VRMS ");
		Read_ADE7953_Register((uint16_t) VRMS, (uint16_t) register_32bit,&dummy_data);
		Result_Data = dummy_data * ADE7953_VRMS_LSB;
		V_data = Result_Data; 
		oem_dtoa(Result_Data,myString,radix_point_size);
		printf(myString);printf("\r\n\r\n");

		printf("Period ");
		Read_ADE7953_Register((uint16_t) PERIOD, (uint16_t) register_16bit,&dummy_data);
		dummy_data++;
		Dtime = (double)dummy_data/223000;
		Freq = 1/Dtime; 
		oem_dtoa(Freq,myString,radix_point_size);
		printf(myString);printf("\r\n\r\n");
		
		
		printf("AWATT ");
		Read_ADE7953_Register((uint16_t) AWATT, (uint16_t) register_32bit,&dummy_data);
		P_data = dummy_data * AWATT_Full_Register_LSB;
		//kwatt += P_data / 1000 / 3600;
		oem_dtoa(P_data,myString,radix_point_size);
		printf(myString);printf("\r\n\r\n");
		//printf("KWh\r\n");oem_dtoa(kwatt,myString,6);printf("\r\n");
		
		printf("AVAR ");
		Read_ADE7953_Register((uint16_t) AVAR, (uint16_t) register_32bit,&dummy_data);
		printf(" %d\r\n\r\n",dummy_data);

		P_data = V_data * I_data;		
		oem_dtoa(P_data,myString,radix_point_size);
		printf("Power : ");
		printf(myString);printf("\r\n\r\n");


		printf("PFA ");
		Read_ADE7953_Register((uint16_t) PFA, (uint16_t) register_16bit,&dummy_data);
		if(dummy_data > 0x7fff)
		{
			temp_lsb =  dummy_data - 0x7fff;
			temp_lsb = temp_lsb * PF_LSB;
			temp_lsb = 1- temp_lsb;
		}
		else
		{
			temp_lsb =	0x7fff-dummy_data;
			temp_lsb = temp_lsb * PF_LSB;	
		}	

		oem_dtoa(temp_lsb,myString,radix_point_size);

		//if(dummy_data > 0x7fff )
		//	printf("-");
		
		printf(myString);printf("\r\n\r\n");
		
		
		//printf("PFB ");
		//Read_ADE7953_Register((uint16_t) cmd_PFB, (uint16_t) register_16bit,&dummy_data);
		//printf("\r\n");

		//printf("AIRMSOS ");
		//Read_ADE7953_Register((uint16_t) cmd_AIRMSOS, (uint16_t) register_24bit,&dummy_data);	

		//printf("BIRMSOS ");
		//Read_ADE7953_Register((uint16_t) cmd_BIRMSOS, (uint16_t) register_24bit,&dummy_data);
		
		//printf("VRMSOS ");
		//Read_ADE7953_Register((uint16_t) cmd_VRMSOS, (uint16_t) register_24bit,&dummy_data);
		
		delay_ms(1000);
	}

}
