/*
 * Downlink.cpp
 *
 * Created: 23-09-2018 10:39:27
 * Author : Shubham Bhardwaj, Siddhi Nagre
 */ 

#define  F_CPU 2000000UL

#include <avr/io.h>
//#include "uart.h"
//#include "common.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>
#include <inttypes.h>
#include <string.h>
#include <avr/interrupt.h>
//#include "uart_8_32.h"
#include "ax25.c"
#include "USART.h"
#include "usart.c"
#include "avr_compiler.h"
#include "pmic_driver.h"
#include "spi_driver.h"
#include "spi_driver.c"
#include "cc112x_spi.h"
//#include "cc112x_serial_mode_reg_config.h"
//#include <uart.c>
//Global Variable
unsigned char data_transmit[61],data_receive[60] ,transmit_enable=0,receive_enable=1,SWITCH[8]="$SWITCH",switch_num=0,transmit_check=0;
unsigned char address[61]="SATELLITE_PRATHAM";
unsigned int pkt_length=48,check=0;
int transmitFlag = 0;

SPI_Master_t spiMasterC;

//#define NUM_BYTES  3
#define USART USARTC0
USART_data_t USART_data;
uint8_t buffer[80];
//uint8_t receiveArray[NUM_BYTES];
USART_data_t USART_data;
char sendArray[61];
char receiveArray[61];
bool success;

//UART baudrate calculation
//#define F_CPU 8000000			// oscillator-frequency in Hz, also needs delay.h header (Defined in Project->Configuration options)
#define UART_BAUD_RATE 9600
//for Asynchronous Normal mode (U2x=0) formula will be following:
#define UART_BAUD_CALC (((F_CPU/(UART_BAUD_RATE*16UL)))-1) //16UL still remains regardless of the frequency of crystal

// ATXmega

#define SPICS	4	//4	// Port E bit 4 (pin): SS->chip select for CC
#define SPIDO	5	//5	// Port E bit 5 (pin): MOSI->data out (data to CC_SI)
#define SPIDI	6	//6	// Port E bit 6 (pin): MISO->data in (data from CC_SO)
#define SPICLK	7	//7	// Port E bit 7 (pin): SCK->clock for CC

// CC1125
#define CC_GPIO0	0	// Port E bit 0 (pin) (pin10 of cc)
#define CC_GPIO2	2   // Port E bit 2 (pin) (pin4 of cc)
#define CC_CSN 		4	//4   // (pin11 of cc):-> chip select
#define CC_SI 		5	//5	  //(pin7 of cc):-> data input
#define CC_SO 		6	//6   // (pin9 of cc)-multiplexed with GDO1: -> data output
#define CC_SCLK		7	//7   // (pin8 of cc):-> clock input

// CC2500/CC1100/CC1101 STROBE, CONTROL AND STATUS REGISTER


/*
#define CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CCxxx0_SYNC1        0x04        // Sync word, high byte
#define CCxxx0_SYNC0        0x05        // Sync word, low byte
#define CCxxx0_PKTLEN       0x06        // Packet length
#define CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define CCxxx0_ADDR         0x09        // Device address
#define CCxxx0_CHANNR       0x0A        // Channel number
#define CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define CCxxx0_FREQ2        0x0D        // Frequency control word, high byte
#define CCxxx0_FREQ1        0x0E        // Frequency control word, middle byte
#define CCxxx0_FREQ0        0x0F        // Frequency control word, low byte
#define CCxxx0_MDMCFG4      0x10        // Modem configuration
#define CCxxx0_MDMCFG3      0x11        // Modem configuration
#define CCxxx0_MDMCFG2      0x12        // Modem configuration
#define CCxxx0_MDMCFG1      0x13        // Modem configuration
#define CCxxx0_MDMCFG0      0x14        // Modem configuration
#define CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define CCxxx0_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define CCxxx0_AGCCTRL2     0x1B        // AGC control
#define CCxxx0_AGCCTRL1     0x1C        // AGC control
#define CCxxx0_AGCCTRL0     0x1D        // AGC control
#define CCxxx0_WOREVT1      0x1E        // High byte Event 0 timeout
#define CCxxx0_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define CCxxx0_FREND1       0x21        // Front end RX configuration
#define CCxxx0_FREND0       0x22        // Front end TX configuration
#define CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define CCxxx0_FSTEST       0x29        // Frequency synthesizer calibration control
#define CCxxx0_PTEST        0x2A        // Production test
#define CCxxx0_AGCTEST      0x2B        // AGC test
#define CCxxx0_TEST2        0x2C        // Various test settings
#define CCxxx0_TEST1        0x2D        // Various test settings
#define CCxxx0_TEST0        0x2E        // Various test settings
*/
// Strobe commands
#define CCxxx0_SRES         0x30        // Reset chip.
#define CCxxx0_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
// If in RX/TX: Go to a wait state where only the synthesizer is
// running (for quick RX / TX turnaround).
#define CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define CCxxx0_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
// (enables quick start).
#define CCxxx0_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
// MCSM0.FS_AUTOCAL=1.
#define CCxxx0_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
// MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
// Only go to TX if channel is clear.
#define CCxxx0_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit

// Wake-On-Radio mode if applicable. - ** IN CC1125,RX NEED NOT BE ON FULL TIME, REFER DATASHEET PAGE 26 **
#define CCxxx0_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CCxxx0_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CCxxx0_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CCxxx0_SWORRST      0x3C        // Reset real time clock.
#define CCxxx0_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
// bytes for simpler software.
// Status registers (read & burst)
#define CCxxx0_PARTNUM      (0x30 | 0xc0) // Part Number of CC1101
#define CCxxx0_VERSION      (0x31 | 0xc0) // Current version Number
#define CCxxx0_FREQEST      (0x32 | 0xc0) // Frequency Offset Estimate
#define CCxxx0_LQI          (0x33 | 0xc0) // Demodulator Estimate for Link Quality
#define CCxxx0_RSSI         (0x34 | 0xc0) // Received signal strength indication
#define CCxxx0_MARCSTATE    (0x35 | 0xc0) // Control State Machine State
#define CCxxx0_WORTIME1     (0x36 | 0xc0) // Hight Byte of Wake-on-Radio time
#define CCxxx0_WORTIME0     (0x37 | 0xc0) // Low Byte of Wake-on-Radio time
#define CCxxx0_PKTSTATUS    (0x38 | 0xc0) // Current GDOx status and packet status
#define CCxxx0_VCO_VC_DAC   (0x39 | 0xc0) // Current setting from PLL calibration madule
#define CCxxx0_TXBYTES      (0x3A | 0xc0) // Underflow and number of bytes in TX FIFO
#define CCxxx0_RXBYTES      (0x3B | 0xc0) // Overflow and number of bytes in RX FIFO

#define CCxxx0_PATABLE      0x3E
#define CCxxx0_TXFIFO       0x3F
#define CCxxx0_RXFIFO       0x3F

//-------------------------------------------------------------------------------------------------------
// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct RF_SETTINGS {
	// Rf settings for CC1101
	unsigned char  SYNC_CFG1;          
	unsigned char  MODCFG_DEV_E;
	unsigned char  FS_CFG;
	unsigned char  FREQ2;
	unsigned char  FREQ1;
	unsigned char  FREQ0;
} RF_SETTINGS;

unsigned char ccxxx0_Strobe(unsigned char);
// Input parameters - adddress of the Command Strobe
//Return parameters - the Chip Status byte

unsigned char ccxxx0_Read(unsigned char);
 // Input parameters - adddress of the Byte to be read.
 //Return parameters - the value of byte stored at that address

unsigned char ccxxx0_Write(unsigned char, unsigned char);
// Input parameters - address of byte to be written, the byte to be written
// Return parameters - the Chip Status byte

void ccxxx0_ReadBurst(unsigned char, unsigned char*, unsigned int);
/** Input parameters - address of first byte to be read,
                       the pointer of the array where we have to store the read values,
											 the number of bytes to be read
    Return parameters - void */

void ccxxx0_WriteBurst(unsigned char, unsigned char*, unsigned int);
/** Input parameters - address of first byte where we have to write,
                       the pointer of the array where the values to be written are stored,
											 the number of bytes to be written
    Return parameters - void */

void ccxxx0_PowerOnReset();
// Power on reset. The manual reset is choosed.
//The exact details of manual reset on page 51 datasheet.

void ccxxx0_Setup();//const RF_SETTINGS*);
// Write all the RF Settings Registers one by one. And echo to Computer using USART

void SPI_Master_Init(void);
void init_UART0(void);
void transmit_USART(char);
void transmit_string_USART(char *);
uint8_t receive_UART0();

void CC_Transmit();
void  CC_Receive();

/*void SPI_Master_Init(void)
{
	DDRC =0x01;
	PORTC=0x01;
	DDRD |= (1<<PD7);
	PORTD |= (1<<PD7);
	init_UART0();

	// SPI register config
	DDRB &= ~(1 << SPIDI);	// set port B SPI data input to input
	DDRB |= (1 << SPICLK) ;	// set port B SPI clock to output
	DDRB |= (1 << SPIDO);	// set port B SPI data out to output
	DDRB |= (1 << SPICS);	// set port B SPI chip select to output
	DDRB &= ~(1 << CC_GDO0);	// set port B packet received pin to input

	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0) ;//| (1 << SPI2X) ;// | (1 << SPR1) ;// | (1 << SPR0);
	SPSR = 0x00;

	PORTB |= (1 << SPICS);	// set chip select to high (CC is NOT selected)
	PORTB &= ~(1 << SPIDO);	// data out =0
	PORTB |= (1 << SPICLK); // clock out =1

}
*/

void SPI_Master_Init()	
{
	// ** Do initialisation to 2 ports for UART **
	
	//SPI Register Configuration
	PORTE.DIRSET = PIN4_bm;																							//Set SS as output
	PORTE.DIRSET = PIN5_bm;																							//MOSI as output
	PORTE.DIRSET = PIN7_bm;																							//SCK as output
	PORTE.PIN4CTRL = PORT_OPC_WIREDANDPULL_gc;																		//Set PullUp at SS 
	// We can't leave PIN4 to a floating state of 0 or 3.3V when in idle state, so we set it to always high being connected to VCC until we explicitly make it low
	PORTE.OUTSET = PIN4_bm;																							//Set SS high for no Slave
	SPI_MasterInit(&spiMasterC,&SPIC,&PORTE,false,SPI_MODE_2_gc,SPI_INTLVL_OFF_gc,true,SPI_PRESCALER_DIV4_gc);		//Initialize device as master (Mode 2, MSB first, 2X speed, prescaler 4)
	// Use Mode 0, Mode 2 refers '10' in binary, false value for first 4, true for next 4, use eg. DIV16 for fosc/16 
	PORTE.OUTSET = PIN7_bm;
	PORTE.OUTCLR = PIN5_bm;
}

/* Deviation = 197.753906 */
/* Base frequency = 433.999786 */
/* Carrier frequency = 433.999786 */
/* Channel number = 0 */
/* Carrier frequency = 433.999786 */
/* Modulated = true */
/* Modulation format = GFSK */
/* Manchester enable = false */
/* Sync word qualifier mode = 30/32 sync word bits detected */
/* Preamble count = 4 */
/* Channel spacing = 199.813843 */
/* Carrier frequency = 433.999786 */
/* Data rate = 2.40111 */
/* RX filter BW = 210.937500 */
/* Data format = Normal mode */
/* CRC enable = true */
/* Whitening = false */
/* Device address = 0 */
/* Address config = No address check */
/* CRC autoflush = false */
/* PA ramping = false */
/* TX power = 10 */

/*RF_SETTINGS rfSettings = {
	0x08;			// SYNC_CFG1;
	0x29;			// MODCFG_DEV_E;
	0x0B;			// FS_CFG;
	0x6D;			// FREQ2;
	0x6C;			// FREQ1;
	0xCB;			// FREQ0;

};
*/


// PATABLE (0 dBm output power) // Page number 60 CC1101 datasheet
unsigned char paTable[] = {
	0x60 //, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0
	//0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03
};



unsigned char ccxxx0_Read(unsigned char addr)			// - DONE
{
	unsigned char x; // The variable where the read Byte is stored
	PORTE.OUTCLR = PIN4_bm; // make the SS pin low to start the communication

	while(PORTE.IN & PIN6_bm);

	SPIC_DATA = (addr | 0x80); // Header byte R/~W bit - 1 Burst bit - 0
	while(!(SPIC_STATUS & PIN7_bm)); // Wait for transmission to be completed
	x = SPIC_DATA; // flush SPDR

// Receiving the byte at addr
	SPIC_DATA = 0;
	while(!(SPIC_STATUS & PIN7_bm));
	x = SPIC_DATA; //// flush SPDR

	PORTE.OUTSET = PIN4_bm; // Make SS high to stop communication

	return x;
}

unsigned char ccxxx0_Write(unsigned char addr, unsigned char dat)			// - DONE
{
	unsigned char x;
	PORTE.OUTCLR = PIN4_bm;	 // make the SS pin low to start the communication

	while(PORTE.IN & PIN6_bm);

	SPIC_DATA = addr; // Header Byte R/~W bit - 0 Burst bit - 0
	while(!(SPIC_STATUS & PIN7_bm));
	x = SPIC_DATA;// flush SPDR

	SPIC_DATA = dat; // data to be written at addr
	while(!(SPIC_STATUS & PIN7_bm));
	x = SPIC_DATA; // get data from SPDR

	PORTE.OUTSET = PIN4_bm; // Make SS high to stop communication

	return x; // The Chip Status Byte
}

unsigned char ccxxx0_Strobe(unsigned char addr)		// - DONE
{
    unsigned char x;
	PORTE.OUTCLR = PIN4_bm;			// make the SS pin low to start the communication

	while(PORTE.IN & PIN6_bm);

    SPIC_DATA = addr;				// The address of the Strobe Command
	while(!(SPIC_STATUS & PIN7_bm));
    x = SPIC_DATA;					// flush SPDR the Status Byte

    PORTE.OUTSET = PIN4_bm;			// Make SS high to stop communication

    return x;
}

void ccxxx0_ReadBurst(unsigned char addr, unsigned char* dataPtr, unsigned int dataCount)		// - DONE
{
	unsigned char x;

	PORTE.OUTCLR = PIN4_bm; // make the SS pin low to start the communication

	while(PORTE.IN & PIN6_bm);

    SPIC_DATA = (addr | 0xc0); // Header Byte R/~W bit - 1 Burst bit - 1
	while(!(SPIC_STATUS & PIN7_bm));
	x = SPIC_DATA;// flush SPDR

	while(dataCount) { // Loop that stops communication when desired number of bytes are read
	    SPIC_DATA = 0;
		while(!(SPIC_STATUS & PIN7_bm));

	    *dataPtr++ = SPIC_DATA; // get data from SPDR, *dataPtr++ points the next element
		dataCount--;
	}

    PORTE.OUTSET = PIN4_bm; // Make SS high to stop communication
}

void ccxxx0_WriteBurst(unsigned char addr, unsigned char* dataPtr, unsigned int dataCount)			// - DONE
{
	unsigned char x;

	PORTE.OUTCLR = PIN4_bm; // make the SS pin low to start the communication

	while(PORTE.IN & PIN6_bm);

    SPIC_DATA = addr | 0x40; // Header Byte R/~W bit - 0 Burst bit - 1
	while(!(SPIC_STATUS & PIN7_bm));
	x = SPIC_DATA; // flush SPDR

	while(dataCount) { // Loop that stops communication after desired number of writing cycles
	    SPIC_DATA = *dataPtr++;
		while(!(SPIC_STATUS & PIN7_bm));

		dataCount--;
	}

    PORTE.OUTSET = PIN4_bm; // Make SS high to stop communication
}

void ccxxx0_PowerOnReset() // Manual Reset		// - DONE
{
	unsigned char x;
	//datasheet cc1101 pg on.51 Manual Reset
    PORTE.OUTSET = PIN4_bm; // Make SS high
	_delay_us(1);								// ** Check how much delay we need in ATXMega corresponding to these functions **
	PORTE.OUTCLR = PIN4_bm; // Make SS low
	_delay_us(1);
    PORTE.OUTSET = PIN4_bm; // Make SS high for atleast 41 us
	_delay_us(41);

	PORTE.OUTCLR = PIN4_bm; // Make SS low

	while(PORTE.IN & PIN6_bm); // wait for SO to go low

	_delay_us(50);

    SPIC_DATA = CC112X_SRES; // Isssue the SRES command strobe
	while(!(SPIC_STATUS & PIN7_bm));
	x = SPIC_DATA; // flush SPDR

	while(PORTE.IN & PIN6_bm); // When SO goes low again, reset is complete

// **THE CHIP IS IDLE STATE AFTER RESET**

	_delay_us(50);

    PORTE.OUTSET = PIN4_bm; // Make SS high
}

void transmit_USART(char data)
{
	uint8_t i;
	PORTC.DIRSET   = PIN3_bm;
	PORTC.DIRCLR   = PIN2_bm;
	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

	/* Enable global interrupts. */
	sei();

	/* Send sendArray. */
	sendArray[0]=data;
	i = 0;
	while (i < 1) {
	bool Tobuffer;
	Tobuffer=	USART_TXBuffer_PutByte(&USART_data, sendArray[i]);
		
	if(Tobuffer){
			i++;
	             }
	}
}

void transmit_string_USART(char *string)
{
	uint8_t i;
	PORTC.DIRSET   = PIN3_bm;
	PORTC.DIRCLR   = PIN2_bm;
	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USART, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
	USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);
	USART_Baudrate_Set(&USART, 12 , 0);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

	/* Enable global interrupts. */
	sei();

	/* Send sendArray. */
	strcpy(sendArray,string);
	i = 0;
	while (i < 61) {
		bool Tobuffer;
		Tobuffer=	USART_TXBuffer_PutByte(&USART_data, sendArray[i]);
		
		if(Tobuffer){
			i++;
		}
	}
}

void ccxxx0_Setup()//const RF_SETTINGS* settings)
{
	unsigned char read;
	// Write register settings
	
	//writeByte = 0x08;
	ccxxx0_Write(CC112X_IOCFG3, 0x08);
	//writeByte = 0x09;
	ccxxx0_Write(CC112X_IOCFG2, 0x09);
	//writeByte = 0x30;
	ccxxx0_Write(CC112X_IOCFG0, 0x30);
	//writeByte = 0x08;
	ccxxx0_Write(CC112X_SYNC_CFG1, 0x08);
	//writeByte = 0x00;
	ccxxx0_Write(CC112X_PREAMBLE_CFG1, 0x00);
	//writeByte = 0x06;
	ccxxx0_Write(CC112X_MDMCFG1, 0x06);
	//writeByte = 0x0A;
	ccxxx0_Write(CC112X_MDMCFG0, 0x0A);
	//writeByte = 0xA9;
	ccxxx0_Write(CC112X_AGC_CFG1, 0xA9);
	//writeByte = 0x05;
	ccxxx0_Write(CC112X_PKT_CFG2, 0x05);
	//writeByte = 0x00;
	ccxxx0_Write(CC112X_PKT_CFG1, 0x00);
	//writeByte = 0x08;
	ccxxx0_Write(CC112X_SERIAL_STATUS, 0x08);
	ccxxx0_Write(CC112X_MODCFG_DEV_E, 0x29);
	ccxxx0_Write(CC112X_FREQ_IF_CFG, 0x50);
	ccxxx0_Write(CC112X_SYMBOL_RATE2, 0x48);
	ccxxx0_Write(CC112X_SYMBOL_RATE1, 0x93);
	ccxxx0_Write(CC112X_SYMBOL_RATE0, 0x75);
	ccxxx0_Write(CC112X_FS_CFG, 0x7E);
	ccxxx0_Write(CC112X_PA_CFG0, 0x7E);
	ccxxx0_Write(CC112X_FREQ2, 0x6D);
	ccxxx0_Write(CC112X_FREQ1, 0x7D);
	ccxxx0_Write(CC112X_FREQ0, 0x81);
	
	/*ccxxx0_Write(CCxxx0_IOCFG0,   settings->IOCFG0); // Write the register value at its address
	read = ccxxx0_Read(CCxxx0_IOCFG0); // Read the written register back and send it through UART
	transmit_USART(read);// Send the read value through UART
	ccxxx0_Write(CCxxx0_FIFOTHR,  settings->FIFOTHR);
	read = ccxxx0_Read(CCxxx0_FIFOTHR);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_PKTCTRL0, settings->PKTCTRL0);
	read = ccxxx0_Read(CCxxx0_PKTCTRL0);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_FSCTRL1,  settings->FSCTRL1);
	read = ccxxx0_Read(CCxxx0_FSCTRL1);
	transmit_USART(read);
	//ccxxx0_Write(CCxxx0_FSCTRL0,  settings->FSCTRL0);
	ccxxx0_Write(CCxxx0_FREQ2,    settings->FREQ2);
	read = ccxxx0_Read(CCxxx0_FREQ2);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_FREQ1,    settings->FREQ1);
	read = ccxxx0_Read(CCxxx0_FREQ1);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_FREQ0,    settings->FREQ0);
	read = ccxxx0_Read(CCxxx0_FREQ0);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_MDMCFG4,  settings->MDMCFG4);
	read = ccxxx0_Read(CCxxx0_MDMCFG4);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_MDMCFG3,  settings->MDMCFG3);
	read = ccxxx0_Read(CCxxx0_MDMCFG3);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_MDMCFG2,  settings->MDMCFG2);
	read = ccxxx0_Read(CCxxx0_MDMCFG2);
	transmit_USART(read);
	//ccxxx0_Write(CCxxx0_MDMCFG1,  settings->MDMCFG1);
	//ccxxx0_Write(CCxxx0_MDMCFG0,  settings->MDMCFG0);
	ccxxx0_Write(CCxxx0_DEVIATN,  settings->DEVIATN);
	read = ccxxx0_Read(CCxxx0_DEVIATN);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_MCSM0 ,   settings->MCSM0 );
	read = ccxxx0_Read(CCxxx0_MCSM0);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_FOCCFG,   settings->FOCCFG);
	read = ccxxx0_Read(CCxxx0_FOCCFG);
	transmit_USART(read);
	//ccxxx0_Write(CCxxx0_BSCFG,    settings->BSCFG);
	ccxxx0_Write(CCxxx0_WORCTRL,  settings->WORCTRL);
	read = ccxxx0_Read(CCxxx0_WORCTRL);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_FSCAL3,   settings->FSCAL3);
	read = ccxxx0_Read(CCxxx0_FSCAL3);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_FSCAL2,   settings->FSCAL2);
	read = ccxxx0_Read(CCxxx0_FSCAL2);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_FSCAL1,   settings->FSCAL1);
	read = ccxxx0_Read(CCxxx0_FSCAL1);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_FSCAL0,   settings->FSCAL0);
	read = ccxxx0_Read(CCxxx0_FSCAL0);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_TEST2,    settings->TEST2);
	read = ccxxx0_Read(CCxxx0_TEST2);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_TEST1,    settings->TEST1);
	read = ccxxx0_Read(CCxxx0_TEST1);
	transmit_USART(read);
	ccxxx0_Write(CCxxx0_TEST0,    settings->TEST0);
	read = ccxxx0_Read(CCxxx0_TEST0);
	transmit_USART(read);*/
}


void CC_Transmit(unsigned int pkt_length)
{
	PORTE.OUT = 0xff;
	char temp[61];

	//strcpy((char*)data_transmit, (const char*)address);
	for (int i = 0; i<61; i++)
	{
		data_transmit[i] =  address[i];
	}

	ccxxx0_Strobe(CC112X_SIDLE);//Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable
	//ccxxx0_WriteBurst(CCxxx0_PATABLE, &paTable[0], 1); // max power
	ccxxx0_Strobe(CC112X_SFTX); // flush tx buff

	ccxxx0_Strobe(CC112X_SFTX); // goto tx mode
	ccxxx0_WriteBurst(CC112X_BURST_TXFIFO, (unsigned char*)data_transmit,61); // addr=M, payload=4 bytes, Total PKTLEN=5//see cc1101 datasheet pg no.-40
	//ccxxx0_ReadBurst(CCxxx0_TXFIFO, (unsigned char*)data_transmit,35);
	if(data_transmit[60]==0x7E){PORTE.OUT = 0xAA;};
	_delay_ms(5);
	//ccxxx0_Strobe(CCxxx0_STX); // goto tx mode
	_delay_ms(75);// initially 120ms

	ccxxx0_Strobe(CC112X_SIDLE);
	_delay_ms(10);
	PORTE.OUT = 0x00;
	//transmit_string_USART("transmitted Data: ");
	//transmit_string_USART((unsigned char *)data_transmit);
	//transmit_string_USART("\r\n");
}


/*ISR(USART_RXC_vect){
	cli(); //disable intterupts
	//transmitFlag = 0;
	int i = 0;
	address[i] = UDR; //
	//if(address[i]==0x7E){PORTC = 0xCC;}

		for(i = 1;i<61; i++){
			char temp = receive_UART0();
			address[i] = temp;}
			//transmit_USART(temp);}
			//if(address[60]==0x7E){PORTC = 0xEE;}
		address[i] = '\0';
		CC_Transmit(61);
		//transmitFlag = 1;
		sei(); // Global interrupts enable
	//transmit_string_USART("In ISR");

	//transmit_string_USART("Out of ISR");
}*/

/*ISR(USARTC0_RXC_vect)
{ int receive=0;
	USART_RXComplete(&USART_data);
	if (USART_RXBufferData_Available(&USART_data)) {                                               // modified by  me
	receive = USART_RXBuffer_GetByte(&USART_data);}                  // receive the data      // modified
	UART_TXBuffer_PutByte(&USART_data, receive);	                     // send data
	
}*/

/*****
	MAIN program
*****/
int main(void)
{
	cli(); 							//Clears the global interrupts
	SPI_Master_Init();
	//sei();

	PORTE.DIR = 0b00001111;
	PORTE.OUT = 0xFF;
	//transmit_string_USART("cc1101_PowerOnReset\r\n");
	_delay_ms(1000);
	PORTE.OUT = 0x00;
	ccxxx0_PowerOnReset();
	//transmit_string_USART("cc1101_Setup\r\n");
	ccxxx0_Setup(); //&rfSettings);
	uint8_t buf[80];
	unsigned char test_data[34];
	memcpy(buf, "IITBOMBAYPRATHAMIITBOMBAYADVITIYXX", 34);
	for(int jk = 0; jk < 34; jk++)
		test_data[jk] = buf[jk];

	make_ax25_frame_from_data((uint8_t *)address, (uint8_t *)test_data);
	address[61] = '\0';
		while(1)
		{
			CC_Transmit(61);
			_delay_ms(2000);
		}


	// LOOP HERE FOREVER .. if we ever come here :)
	while (1) { asm("nop"); }

	return 0;
}

