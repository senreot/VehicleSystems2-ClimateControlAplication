/*MIT License

Copyright (c) 2016 senreot

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "global.h"
#include "delay.h"
#include "lcd.h"
#include "usart.h"

#define MAX_FAN_CTRL 16000
#define MIN_FAN_CTRL 0

#define MAX_TEMP 30
#define MIN_TEMP 15

#define MAX_POS_OCR 2300//1900 //The MAX value of the OCR corresponding with Max position of the servo
#define MIN_POS_OCR 500//1100  //The MIN value of the OCR corresponding with Min position of the servo

#define FLAP_POS_R 2300 //Ventilation flap WINDSHIELD position
#define FLAP_POS_C 1400 //Ventilation flap CENTER position
#define FLAP_POS_L 500 //Ventilation flap FLOOR position

#define LM77_ADDR  0x48  //This is the address of the temperature sensor on TWI
#define REAL_TIME_CLK_ADDR 0xD0 //This is the address of the serial real time clock

#define DDR_SPI DDRB
#define DD_MISO PB3
#define DD_MOSI PB2
#define DD_SCK PB1
#define DD_SS PB0

volatile unsigned char buttons;			//Allocate a byte that will record the input of Port C, it will be accessed by the main function as well as the interrupt service routine
volatile unsigned char bToggle = 0;		//Allocate an oversize boolean to record that a button has been pushed or released

volatile unsigned int icp, icp_0;	//Input capture register
volatile unsigned int overflow1=1;	// Count the Timer/Counter1 overflows

volatile unsigned int overflow3=0;		// Count the Timer/Counter3 overflows

volatile char pot_mux = 0;		//Represent the ADC channel selected
volatile unsigned int pot1 = 0;		// Potetiometer1
volatile unsigned int pot2 = 0;		// Potetiometer2

volatile unsigned int rpm = 0;		//RPM received by CAN

int portSetup(void);
int interruptSetup(void);
int displaySetup(void);
void randomFill(void);
void updateLED(char* LED);
void TimerCounter1setup(void);
void TimerCounter3setup(void);
void ADCsetup(void);
int sendInfoToComputer(volatile unsigned int* pot1, volatile unsigned int* pot2, volatile unsigned int* rpm );
void TWIsetup(void);
int TWI_masterReceiverMode(const char addrs, char* msg, unsigned int msg_len);
void SPI_MasterInit(void);
void SPI_MasterTransmit(uint8_t rwAddress,uint8_t cData);

int main(void)
{
	char LED;
	char isAutomatic = 0xFF;
	char mode_char;

	unsigned int flap_pos = FLAP_POS_C;
	char flap_dir = 0x00; //0xFF means the flap is going up, 0x00 means the flap is going down

	float fan_ctrl;
	char temp[2];
	char temp_disp[5];
	float temperature, temperature_sp = 20.0, temperature_err_0 = 0, temperature_err = 0, temperature_I = 0, temperature_I_0 = 0;
	float Kp , Ki, dt;

	unsigned int LCD_dim;
	float K_dimmer=0.5;

	portSetup();
	interruptSetup();
	ADCsetup();
	ADCSRA |= (1<<ADSC);  //Start ADC
	usart1_init(51); //BAUDRATE_19200 (look at page s183 and 203)

	TWIsetup();

	// Set up SPI and accelerometer
	SPI_MasterInit();
	SPI_MasterTransmit(0b10101100,0b00000101); //accelerometer initialization
	// Write at address, 0x16 = 0b00010110 shifted one to the left and the MSB is 1 to write
	// --,DRPD,SPI3W,STON,GLVL[1],GLVL[0],MODE[1],MODE[0]
	// GLVL [1:0] --> 0 1 --> 2g range, 64 LSB/g
	// MODE [1:0] --> 0 1 --> measurement mode
    
	//PID settings
	Kp = 0;
	Ki = 0;
	dt = 100;
	displaySetup();
	//randomFill();
	mode_char = 'A';
	lcdGotoXY(15, 0);
	lcdPrintData(&mode_char,1);

	sei(); //Enable global interrupts

    while (1) 
    {

		TWI_masterReceiverMode( LM77_ADDR, temp, 2);
		temp[1] = (temp[1]>>3) + ((temp[0]<<5) & !0x07);
		temperature= (float)temp[1] + 0.5*(temp[1]&0x01);

		sprintf(temp_disp,"%d",temp[1]);		//Convert the unsigned integer to an ASCII string
		
		lcdGotoXY(3-(strlen(temp_disp)), 0);		//Position the cursor on
		lcdPrintData(temp_disp, strlen(temp_disp)); //Display the text on the LCD
		if (temp[1]&0x01)
		lcdPrintData(".5/",3);
		else
		lcdPrintData(".0/",3);
		/*When trying to print floating point numbers of type float using sprintf() or printf() functions in an AVR 8-bit C program using Atmel Studio 7, 
		the number does not print correctly. Instead of the float being printed to a string or standard output, a question mark is printed. 
		The reason that floating point numbers are not printed is because the default settings in Atmel Studio disable them for sprintf / printf type functions 
		to save microcontroller memory.
		The solution to this problem is to change some linker settings so that the floating point number is printed as expected.
		(source: https://startingelectronics.org/articles/atmel-AVR-8-bit/print-float-atmel-studio-7/)

		As long as the temperature only needs one decimal I propose a more rudimentary solution. To use two integers, one for each side of the point*/

		temp[1] = (int)temperature_sp; //An int cast to a float always round down
		temp[0] = (temperature_sp - temp[1])*10;

		sprintf(temp_disp,"%d.%dC",temp[1],temp[0]);
		lcdPrintData(temp_disp, strlen(temp_disp)); //Display the text on the LCD
	//
	///********** Temperature control **********/
		////PID
		//temperature_err = temperature - temperature_sp;
		//temperature_I = (temperature_err - temperature_err_0)*dt;
		//fan_ctrl = Kp*temperature_err + Ki*temperature_I;
//
		////Saturation and anti windup
		//if(fan_ctrl>MAX_FAN_CTRL)
		//{
			//fan_ctrl = MAX_FAN_CTRL;
			//temperature_I = temperature_I_0;
		//}
		//else if (fan_ctrl < MIN_FAN_CTRL)
		//{
			//fan_ctrl = MIN_FAN_CTRL;
			//temperature_I = temperature_I_0;
		//}

	/**************** A/M Mode ****************/
		if((buttons == 0b10000000) && bToggle) isAutomatic = ~isAutomatic; //Change the mode if S5 is pressed

		//Automatic
		if(isAutomatic)
		{
			if((buttons == 0b10000000) && bToggle)
			{
				temperature_sp = 20; //Do this just once after the automatic mode is enable
				mode_char = 'A';
				lcdGotoXY(15, 0); 
				lcdPrintData(&mode_char,1);
			}

			if(overflow3>=250) //Timer/Counter3 takes 20ms to overflow, 250 x 20ms = 5s
			{
				if((flap_pos == FLAP_POS_L) || (flap_pos == FLAP_POS_R)) 
				{
					flap_pos = FLAP_POS_C;
					flap_dir = ~flap_dir;
				}

				else if (flap_dir) flap_pos = FLAP_POS_L;
				else flap_pos = FLAP_POS_R;

				overflow3 = 0;
			}
			bToggle = 0;
		}

		//Manual
		else
		{
			if (bToggle)		//This is set to true only in the interrupt service routine at the bottom of the code
			{
				switch(buttons & 0b11111000)
				{
					case 0b10000000:
					mode_char = 'M';
					lcdGotoXY(15, 0);
					lcdPrintData(&mode_char,1);
					break;

					case 0b01000000:			//S4  upper button
					if(temperature_sp < MAX_TEMP) temperature_sp += 0.5;	//Raise the temperature by 0.5�C
					break;

					case 0b00100000:			//S3 left button
					if(flap_pos == FLAP_POS_R) flap_pos = FLAP_POS_C;	//Move the ventilation directional flap to the floor
					else if (flap_pos == FLAP_POS_C) flap_pos = FLAP_POS_L;
					break;

					case 0b00010000:			//S2 lower button
					if(temperature_sp > MIN_TEMP) temperature_sp -= 0.5;	//Lower the temperature by 0.5�C
					break;

					case 0b00001000:			//S1 right button
					if(flap_pos == FLAP_POS_L) flap_pos = FLAP_POS_C;	//Move the ventilation directional flap to the windshield
					else if (flap_pos == FLAP_POS_C) flap_pos = FLAP_POS_R;
					break;

					default:

					break;
				} bToggle = 0;
			}

			////Adjust LCD brightness
			//TIMSK1 &= ~(1<<ICIE1) & ~(1<<TOIE1); //Input Capture interrupt and Overflow interrupt disable
			//if(icp > icp_0) LCD_dim = (icp - icp_0)*overflow1;
			//else LCD_dim = ((icp + 65536) - icp_0)*overflow1;
			//LCD_dim = (int)(LCD_dim * K_dimmer); //Convert the time difference value to an OCR adjusted value
			//OCR1CH = LCD_dim >> 8;
			//OCR1CL = LCD_dim & 0xFF;
			//overflow1 = 1;
			//TIMSK1 |= (1<<ICIE1) | (1<<TOIE1); //Input Capture interrupt and  Overflow interrupt enable
		}
		sendInfoToComputer(&pot1,&pot2,&rpm);
    }
}

int portSetup(void)
{
	//Set up input output direction on Port C and G
	DDRC = 0b00000111;	//Set the Port C's direction to output on the 3 least significant bits and input on the 5 higher ones
	DDRG |= 0b00000011;   //set the Port G's lower 2 bytes to output (LEDs 1 & 2)
	return(0);
}

int interruptSetup(void)
{
	//Set up external Interrupts
	// The five Switches are ORed to Pin PE6 which is alternatively Int6
	EICRB |= (1<<ISC61) | (1<<ISC60);  //The rising edge of INTn generates asynchronously an interrupt request.
	EIMSK |= (1<<INTF6);
	return(0);
}

int displaySetup(void)
{
	// Turn on the back light of the LCD (pin 7 of port B)
	DDRB |= (1<<7);
	PORTB |= (1<<7);

	// Set up display
	lcdInit();		//initialize the LCD
	lcdClear();		//clear the LCD
	lcdHome();		//go to the home of the LCD
	return(0);
}

void randomFill(void)
{
	/*This function fills the LCD with random numbers and capital letters.
	When the screen is complete it cleans the screen and show up a "Welcome" message.*/

	unsigned long matrix = 0x00000000; //Each bit represents a position in the LCD, 1 means the position is already taken
	unsigned long new_matrix = 0x00000000;
	char new ='0';
	unsigned char y=0;
	unsigned char x=0;

	char isOk = 0;
	
	while(matrix != 0xFFFFFFFF)
	{

		//Keep choosing positions until it finds one it is available
		while(isOk!=1)
		{
			y=rand()%2;
			x=rand()%16;
			new_matrix|=((long long)0x01 << x)<<(y*16);
			if(matrix != new_matrix)isOk=1;
		}
		matrix = new_matrix;

		//Randomly choose if the new character is a number or a letter
		new = rand()%2;
		if(new==0)new = '0';
		else new = 'A';

		if(new==48)new += (unsigned char)((double)rand() / ((double)RAND_MAX + 1) * 10);//Choose a random number;
		else new += (unsigned char)((double)rand() / ((double)RAND_MAX + 1) * 25);//Choose a random letter;

		lcdGotoXY(x, y);
		lcdPrintData(&new,1);
		delay_ms(75);

		isOk=0;

	}

	//Finish the animation showing a nice "Welcome" message
	delay_ms(250);
	lcdClear();
	delay_ms(250);
	lcdGotoXY(4, 0);
	lcdPrintData("Welcome", 7);
}

void updateLED( char* LED)
{
	char portc_char, portg_char;

	portc_char = (*LED&0x01)<<2;
	portc_char |= (*LED&0x02);
	portc_char |= (*LED&0x04)>>2;

	portg_char = (*LED&0x08)>>2;
	portg_char |= (*LED&0x10)>>4;

	PORTC=portc_char;
	PORTG=portg_char;

}

void TimerCounter1setup(void)
{
	// The input capture pin is on JP14 near the relay on the development board.
	//Page 137 or 138
	TCCR1A = 0;				//Timer/Counter1 Control Register A is not really used
	TCCR1B = (1<<ICNC1);	//noise canceler
	TCCR1B |= (1<<ICES1);	//rising edge on ICP1 cause interrupt
	TCCR1B |= (0<<CS12)| (0<<CS11)| (1<<CS10); //NO pre-scaler,
	TIFR1 |= (1<<ICF1);		//Set input capture flag
	TCNT1 = 0;				//Timer Counter 1
	TIMSK1 |= (1<<ICIE1) | (1<<TOIE1); //Timer Interrupt Mask Register,Input Capture Interrupt and T/C overflow Interrupt Enable
}

void TimerCounter3setup(void)
{
	//Setup mode on Timer counter 3 to PWM phase correct (OCR3A as TOP)
	TCCR3B = (1<<WGM33) | (0<<WGM32);
	TCCR3A = (1<<WGM31) | (1<<WGM30);
	//Set OC3C on compare match when counting up and clear when counting down
	TCCR3A |= (1<<COM3C1) | (0<<COM3C0);
	//Setup pre-scaller /8 for timer counter 3
	TCCR3B |= (0<<CS32) | (1<<CS31) | (0<<CS30);  //Source clock IO, prescaling /8
	//Set TOP to 16000 -> T=20ms
	OCR3AH=0x4E;
	OCR3AL=0x20;
	//Set OCR3C = 1400 = pos=0� -> t=1.5ms
	OCR3CH = 0x05;
	OCR3CL = 0x78;
	TIMSK3 |= (1<<TOIE3); //Timer Interrupt Mask Register,T/C overflow Interrupt Enable
	
}

void ADCsetup(void)
{
	//Set up analog to digital conversion (ADC)
	//ADMUX register
	//AVcc with external capacitor on AREF pin (the 2 following lines)
	ADMUX &= ~(1<<REFS1);  //Clear REFS1 (although it should be 0 at reset)
	ADMUX |= (1<<REFS0);   //Set REFS0
	ADMUX &= (0b11100000); //Single ended input on ADC0
	ADMUX &= ~(1<<ADLAR);  //Making sure ADLAR is zero (somehow it was set to 1)
	//The ACDC control and status register B ADCSRB
	ADCSRB &= ~(1<<ADTS2) & ~(1<<ADTS1) & ~(1<<ADTS0);  //Free running mode
	//The ADC control and status register A ADCSRA
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) |(1<<ADPS0);//set sampling frequency pre-scaler to a division by 128
	ADCSRA |= (1<<ADEN) | (1<<ADATE) | (1<<ADIE);//enable ADC, able ADC auto trigger, enable ADC interrupt
}

int sendInfoToComputer(volatile unsigned int* pot1, volatile unsigned int* pot2, volatile unsigned int* rpm )
{
	//Dynamic memory allocation
	//char* text = (char*) malloc(1 + 6 + 4 + 8 + 4 + 7 + 4); //Pot1: xxxx, Pot2: xxxx, RPM: xxxx
	//char* value = (char*) malloc(4); 
	char text[35];
	char value[4];
	if(text == 0 || value == 0) return -1; //Out of memory
	strcpy(text,"Pot1: ");
	sprintf(value,"%d",*pot1);
	strcat(text,value);
	strcat(text,", Pot2: ");
	sprintf(value,"%d",*pot2);
	strcat(text,value);
	strcat(text,", RPM: ");
	sprintf(value,"%d",*rpm);
	strcat(text,value);
	strcat(text,"\n");
	for(int i = 0; i<strlen(text); i++) usart1_transmit(text[i]);
	
	//Free the memory
	//free(text);
	//free(value);
	return 0;
}

void TWIsetup(void)
{
	//Setting up TWI baud rate to 100kHz
	TWBR = 72;		//Look at formula on page 210;
	TWSR &= ~(1<<TWPS1) & ~(1<<TWPS0); //With no pre-scaler
}

int TWI_masterReceiverMode(const char addrs, char* msg, unsigned int msg_len)
{
	char status;
	
	//Master receive mode, follow instruction on page 222 of the AT90CAN128 Data sheet
	//Send start condition
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	//Wait for TWINT flag to be set (Start has been transmitted)
	while ( !(TWCR & (1<<TWINT)));
	//Check the value of the TWI status register, making the pre-scaler
	status = TWSR;
	if ((status & 0xF8) != 0x08) return -1;	//We are master of the bus

	//Load device address to TWI data register
	TWDR = ((addrs << 1)| 0x01 ); //Shift the 7 bit address and or it with the read bit
	TWCR = (1 << TWINT) | (1 << TWEN); //Clear TWINT to start the transmission
	while (!(TWCR & (1<<TWINT)));  //Wait for TWINT flag to be set which indicates that the address was sent and acknowledged
	status = TWSR;
	if ((status & 0xF8) != 0x40) return -1;  //SLA+R has been sent and acknowledge has been received

	for (int i=0; i < msg_len; i++)
	{
		TWCR = (1 << TWINT) | (1 << TWEN) | (1<<TWEA); //Clear TWINT to start the reception of the next byte
		//enable acknowledge for the first byte.
		while (!(TWCR & (1<<TWINT)));  //Wait for TWINT flag to be set which indicates that the address was sent and acknowledged
		status = TWSR;
		if ((status & 0xF8) == 0x58) break;  //Data byte has been received and ACK has been sent

		*(msg + i) = TWDR; 
	}

	//TWCR = (1 << TWINT) | (1 << TWEN) ; //Clear TWINT to start the reception of the second byte
	//while (!(TWCR & (1<<TWINT)));  //Wait for TWINT flag to be set which indicates that the address was sent and acknowledged
	//status = TWSR;
	//if ((status & 0xF8) != 0x58) return -1;  //Data byte has been received and NACK has been sent, last byte transmited
//
	//*(msg + msg_len -1) = TWDR;	

	//Transmit STOP condition
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	return 0;
}

void SPI_MasterInit(void)
{
	/* Set MOSI and SCK output, all others input */
	DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS); //Master out Slave in (MOSI) is an output, Clock is an output, Slave select at output(this limits the master to be both but OK for here)
	DDR_SPI &= ~(1<<DD_MISO);	//Master in Slave out (MISO) is an input
	PORTB |= (1<<DD_SS);		//set the slave select high: idle
	/* Enable SPI, Master, set clock rate fck/64 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR1);
}

void SPI_MasterTransmit(uint8_t rwAddress,uint8_t cData)
{
	/*NOTE: The command byte of SPI bus is:
	(MSB) R/W A A A A A A 0 (LSB)
	|  |         | |
	|  |         |  - Don't care bit
	|   ------------- Register address,
	---------------- Read/Write bit (1=write, 0=read)*/

	/* Start transmission */
	SPDR = rwAddress;
	PORTB &= ~(1<<DD_SS); //start transmission by pulling low the Slave Select line
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));

	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)));

	PORTB |= (1<<DD_SS); //Return to idle mode
}


ISR(INT6_vect)  //Execute the following code if an INT6 interrupt has been generated
{
	bToggle = 1;		//Make a record that a button has been pushed or released
	buttons = PINC;		//Make a record of what the input of the Port C looked like
	buttons &= 0b11111000;
}

ISR(ADC_vect)
{
ADCSRA &= ~(1<<ADIE);      //disable ADC interrupt to prevent value update during the conversion

//ADCL must be read first, then ADCH, to ensure that the content of the Data Registers belongs to the same conversion
	if(pot_mux == 0)	
	{
		pot1 = ADCL;	   //Load the low byte of the ADC result
		pot1 += (ADCH<<8); //shift the high byte by 8bits to put the high byte in the variable
		ADMUX &= (0b11100000); //Single ended input on ADC0
		pot_mux = 1;
	}

	else if(pot_mux == 1)
	{
		pot2 = ADCL;	   //Load the low byte of the ADC result
		pot2 += (ADCH<<8); //shift the high byte by 8bits to put the high byte in the variable
		ADMUX |= (0b00000001); //Single ended input on ADC1
		pot_mux = 0;
	}
ADCSRA |= (1<<ADIE);      //re-enable ADC interrupt
}

ISR(TIMER1_CAPT_vect)
{
	icp_0 = icp;
	icp = ICR1;
}

ISR(TIMER1_OVF_vect)
{
	overflow1++;
}

ISR(TIMER3_OVF_vect)
{
	overflow3++;
}