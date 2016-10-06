
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#include "global.h"
#include "delay.h"
#include "lcd.h"
#include "usart.h"

#define MAX_FAN_CTRL 100
#define MIN_FAN_CTRL 0

#define MAX_POS_OCR 2300//1900 //The MAX value of the OCR corresponding with Max position of the servo
#define MIN_POS_OCR 500//1100  //The MIN value of the OCR corresponding with Min position of the servo

#define FLAP_POS_R 2300 //Ventilation flap WINDSHIELD position
#define FLAP_POS_C 1400 //Ventilation flap CENTER position
#define FLAP_POS_L 500 //Ventilation flap FLOOR position

volatile unsigned char buttons;			//Allocate a byte that will record the input of Port C, it will be accessed by the main function as well as the interrupt service routine
volatile unsigned char bToggle = 0;		//Allocate an oversize boolean to record that a button has been pushed or released

volatile unsigned int overflow3=0;		// Count the Timer/Counter3 overflows

volatile unsigned int pot1 = 0;		// Potetiometer1
volatile unsigned int pot2 = 0;		// Potetiometer2
volatile unsigned int rpm = 0;		//RPM received by CAN

void randomFill(void);
void updateLED(char* LED);
void TimerCounter3setup(void);
int sendInfoToComputer(volatile unsigned int* pot1, volatile unsigned int* pot2, volatile unsigned int* rpm );

int main(void)
{
	char LED;
	char isAutomatic = 0xFF;

	unsigned int flap_pos;
	char flap_dir; //0xFF means the flap is going up, 0x00 means the flap is going down

	float fan_ctrl;
	float temperature, temperature_sp, temperature_err_0, temperature_err, temperature_I, temperature_I_0;
	float Kp, Ki, dt;

	usart1_init(51); //BAUDRATE_19200 (look at page s183 and 203)
    
    while (1) 
    {

	/********** Temperature control **********/
		//PID
		temperature_err = temperature - temperature_sp;
		temperature_I = (temperature_err - temperature_err_0)*dt;
		fan_ctrl = Kp*temperature_err + Ki*temperature_I;

		//Saturation and anti windup
		if(fan_ctrl>MAX_FAN_CTRL)
		{
			fan_ctrl = MAX_FAN_CTRL;
			temperature_I = temperature_I_0;
		}
		else if (fan_ctrl < MIN_FAN_CTRL)
		{
			fan_ctrl = MIN_FAN_CTRL;
			temperature_I = temperature_I_0;
		}

	/**************** A/M Mode ****************/
		if((buttons == 0b10000000) && bToggle) isAutomatic = ~isAutomatic; //Change the mode if S5 is pressed

		//Automatic
		if(isAutomatic)
		{
			if((buttons == 0b10000000) && bToggle) temperature_sp = 20; //Do this just once after the automatic mode is enable

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
		}

		//Manual
		else
		{
			if (bToggle)		//This is set to true only in the interrupt service routine at the bottom of the code
			{
				switch(buttons & 0b11111000)
				{
					case 0b01000000:			//S4  upper button
					temperature_sp += 0.5;	//Raise the temperature by 0.5ºC
					break;

					case 0b00100000:			//S3 left button
					if(flap_pos == FLAP_POS_R) flap_pos = FLAP_POS_C;	//Move the ventilation directional flap to the floor
					else if (flap_pos == FLAP_POS_C) flap_pos = FLAP_POS_L;
					break;

					case 0b00010000:			//S2 lower button
					temperature_sp -= 0.5;	//Lower the temperature by 0.5ºC
					break;

					case 0b00001000:			//S1 right button
					if(flap_pos == FLAP_POS_L) flap_pos = FLAP_POS_C;	//Move the ventilation directional flap to the windshield
					else if (flap_pos == FLAP_POS_C) flap_pos = FLAP_POS_R;
					break;

					default:

					break;
				} bToggle = 0;
			}

			sendInfoToComputer(&pot1,&pot2,&rpm);
		}
    }
}

void randomFill(void)
{
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
	//Set OCR3C = 1400 = pos=0º -> t=1.5ms
	OCR3CH = 0x05;
	OCR3CL = 0x78;
	
}

int sendInfoToComputer(volatile unsigned int* pot1, volatile unsigned int* pot2, volatile unsigned int* rpm )
{
	//Dynamic memory allocation
	char* text = (char*) malloc(1 + 6 + 4 + 8 + 4 + 7 + 4); //Pot1: xxxx, Pot2: xxxx, RPM: xxxx
	char* value = (char*) malloc(4); 
	if(text == 0 || value == 0) return -1; //Out of memory
	strcpy(text,"Pot1: ");
	itoa(*pot1,value,4);
	strcat(text,value);
	strcat(text,", ");
	itoa(*pot2,value,4);
	strcat(text,", ");
	itoa(*rpm,value,4);
	strcat(text,value);
	strcat(text,"\n");
	
	for(int i = 0; i<strlen(text); i++) usart1_transmit(text[i]);
	
	//Free the memory
	free(text);
	free(value);
	return 0;
}

ISR(INT6_vect)  //Execute the following code if an INT6 interrupt has been generated
{
	bToggle = 1;		//Make a record that a button has been pushed or released
	buttons = PINC;		//Make a record of what the input of the Port C looked like
}

ISR(ADC_vect)
{
	pot1 = ADCL;	   //Load the low byte of the ADC result
	pot1 += (ADCH<<8); //shift the high byte by 8bits to put the high byte in the variable
}

ISR(TIMER1_CAPT_vect)
{

}

ISR(TIMER1_OVF_vect)
{

}

ISR(TIMER3_OVF_vect)
{
	overflow3++;
}