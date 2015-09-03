//=========================================================================================
//
// CYTRON TECHNOLOGIES - ROBOT. HEAD to TOE
// ----------------------------------------
//
// DOCUMENT		:	Sample source code to control 2 SD02B through 1 X hardware UART & 
//					1 X software UART using Microchip MPLAB C18 C Compiler Libraries.
//
// MICROCONTROLLER	: Microchip PIC18F4520 (40-pin)
//					  (FOSC = 20MHz external oscillator)
//
// C COMPILER	: * Microchip MPLAB C18 C Compiler Student Edition
//				  * User must intall the compiler in order to compile this source code.
//				  * The free MPLAB C Compiler for PIC18 MCUs Student Edition can be obtained
//					from Microchip official website.
//				  * Currently it can be found at "Design"\"Development Tools"\"Compilers".
//					Register for an account in Microchip website and you can download the
//					C18 C compiler student edition for free.
//
// UARTs 		: * 1 X Hardware UART module (built in PIC)
//				  * 1 X Software UART generated using C18 libraries
//				  * Baudrate = 9600 (both hardware and software UARTs)
//
// TESTED WITH	: Cytron SK40C Enhanced 40 pins PIC Start-up Kit
//
// DESCRIPTION	: * Most of the mid-range and lower level PICs only have one hardware UART.
//				  * So this software gives an idea how to use one hardware UART and one 
//					software UART to control two SD02B Enhanced 2Amp Stepper Motor Drivers.
//				  * Any changes in the source code and hardware condition may affect its 
//					functionality. For example software UART's baudrate is merely depend on
//				 	system clock frequency (FOSC). For FOSC other than 8MHz (internal osc),
//					user needs to recalculate the appropriate delays for this software UART
//					using the formulas given in MPLAB C18 C Compiler Libraries.
//				  * Always refer to MPLAB C Compiler Libraries and SD02B user's manual for 
//					further info.
//				  * This software is not fully tested. Thus, there is no guarantee on the 
//					full functional of this software in all conditions.
//
// WEBSITE		: 	www.cytron.com.my
//
// Update 		:   23 August 2010 - Add in the request encoder function
//
//-----------------------------------------------------------------------------------------
		
//=========================================================================================
// Include
// * User includes the header files (XXXX.h) here.
// * Replace <p18f4520.h> with the header file for any PIC which is chosen by the user.
// * "sw_uart.h" & "usart.h" must be included here to use the C18 UARTs libraries.
//=========================================================================================
#include <p18f4520.h>
#include "sw_uart.h"
#include "usart.h"


//=========================================================================================
// Configuration bits
// * User is advised not to change the settings here
//=========================================================================================
#pragma config OSC = HS			//External oscillator = 20MHz
#pragma config FCMEN = OFF 		//Fail-Safe Clock Monitor disabled
#pragma config IESO = OFF 		//Oscillator Switchover mode disabled
#pragma config PWRT = OFF 		//PWRT disabled
#pragma config BOREN = OFF 		//Brown-out Reset disabled in hardware and software
#pragma config WDT = OFF 		//WDT disabled (control is placed on the SWDTEN bit)
#pragma config MCLRE = ON		//MCLR pin enabled; RE3 input pin disabled
#pragma config PBADEN = OFF 	//PORTB<4:0> pins are configured as digital I/O on Reset
#pragma config DEBUG = OFF		//Background debugger disable
#pragma config XINST = OFF		//Instruction set extension and Indexed Addressing mode disabled
#pragma config LVP = OFF 		//Single-Supply ICSP disabled


//=========================================================================================
// Definitions
//=========================================================================================
#define	led1		LATBbits.LATB6			//define led
#define	led2		LATBbits.LATB7			//define led
#define	sw1			PORTBbits.RB0			//define switch
#define	sw2			PORTBbits.RB1			//define switch

#define	TX_hard		LATCbits.LATC6			//hardware UART TX pin
#define	RX_hard		PORTCbits.RC7			//hardware UART RX pin
#define	TX_soft		LATBbits.LATB4			//software UART TX pin
#define	RX_soft		PORTBbits.RB5			//software UART RX pin


//=========================================================================================
// Global Variables
// * User declares the global variables used in the source code here.
//=========================================================================================
unsigned char 	mode=0;


//=========================================================================================
// Function Prototypes
// * User needs to include all the function prototypes for functions written in this source
//	 code here.
// * No need to include function prototypes for C18 C Compiler Libraries here.
//=========================================================================================
void ISRHigh(void);
void ISRLow(void);
void delay(unsigned long data);
void init(void);

void DelayTXBitUART(void);		
void DelayRXHalfBitUART(void);	
void DelayRXBitUART(void);		

void On(void);
void Off(void);
void Forward(unsigned char lspeed, unsigned char rspeed);
void Backward(unsigned char lspeed, unsigned char rspeed);
void Brake(void);
void Encoder_R(unsigned int encoder);
void Encoder_L(unsigned int encoder);
void Accelerate_R(unsigned char initial_speed, unsigned char final_speed, unsigned char rate);
void Accelerate_L(unsigned char initial_speed, unsigned char final_speed, unsigned char rate);
void Microstep(unsigned char step);

unsigned int Check_Enc_L(void);
unsigned int Check_Enc_R(void);
//=========================================================================================
// Main Function
// * This is the main function where program starts to execute 
//=========================================================================================
void main(void)
{
//-----------------------------------------------------------------------------------------
//	Initialization
//-----------------------------------------------------------------------------------------
	unsigned int encoder_L=0, encoder_R=0;
	init();	
	Off();									//off motors

//-----------------------------------------------------------------------------------------
//	Program starts
//-----------------------------------------------------------------------------------------
	while(1)
	{
	

		if(!sw1)
		{
			mode++;	
			while(!sw1)							//take action after sw1 is released
			{
				led1=1;							//led1 lights up while sw1 is pressed
			}
			delay(10000);						//short delay to debounce the switch
			led1=0;								//led1 turns off when sw1 is released

			switch(mode)
			{
				case 1:	On();					//On motors
						On();					//On motors
						Brake();				//Brake motors
						break;

				case 2:	Forward(20,20);			//Forward with speed = 20 for both motors
						break;

				case 3:	Backward(20,20);		//Backward with speed = 20 for both motors
						break;

				case 4:	Brake();				//Brake motors
						break;					//Shaft is locked at certain angle.

				case 5:	Forward(50,50);			//Forward with speed = 50 for both motors
						break;

				case 6:	Off();					//Off Motors. Try to rotate the shaft again.
						break;					//Shaft is loose.

				case 7:	On();					//On motors first.
						Forward(80,20);			//Forward with two different speeds
						break;

				case 8:	Encoder_R(3000);		//Track right motor at encoder=3000
						Encoder_L(3000);		//Track left motor at encoder=3000
						break;					//Please wait for the motors to stop.
												//You'll see the motor runs at higher speed 
												//will stop first.

				case 9 :Accelerate_R(1,100,150);//Accelerate right motor
						break;					//initial speed=1, final speed=100, rate=150
												//Please wait until the motor runs stable at
												//a speed.

				case 10:Accelerate_L(10,80,70);	//Accelerate left motor
						break;					//initial speed=10, final speed=80, rate=70
												//Please wait until the motor runs stable at
												//a speed.

				case 11:Forward(10,10);			//Set to lower speed
						break;

				case 12:Microstep(1);			//Disable microstepping
						break;

				case 13:Microstep(2);			//Enable 1/2 micro-stepping.
						break;

				case 14:Microstep(5);			//Enable 1/5 micro-stepping.
						break;

				case 15:Microstep(10);			//Enable 1/10 micro-stepping.
						break;

				case 16:Microstep(1);			//Disable microstepping
						break;

				default:;						//if mode>18, do nothing
			}
		}
		if(!sw2)
		{

			while(!sw2);
			On();								//On motors
			//
			// The motors will continuosly rotate on CW and CCW in the infinity loop
			// Until the reset button is pressed
			//
			while(1)							//Infinity loop
			{
				Forward(50,50);					//Forward with speed = 50 for both motors
				while(BusyUSART());
				WriteUSART('R');				//Reset encoder value first before start tracking.		
				encoder_R=0;					//Clear the encoder register
				while(encoder_R<8000)			//check the Right Motor's encoder value
				{
					encoder_R=Check_Enc_R();	//Get the Right Motor's encoder value
				}
				
				Backward(50,50);
				WriteUART('R');					//Reset encoder value first before start tracking.		
				encoder_L=0;					//Clear the encoder register
				while(encoder_L<8000)			//check the Left Motor's encoder value
				{
					encoder_L=Check_Enc_L();	//Get the Left Motor's encoder value
				}
			
			}
		}

	}
}


//=========================================================================================
// Functions
// * The functions for this program are written here.
//=========================================================================================

//-----------------------------------------------------------------------------------------
// * This is the initialization for the program.
// * User can add in more initialization codes here.
// * This function must be called at the beginning of the main program.
//-----------------------------------------------------------------------------------------
void init(void)
{
	ADCON1 = 0b00001111;	//Vdd & Vss, all pins configure as digital I/O
							    
	TRISB = 0b00100011;		//software UART pins: RB5=RX, RB4=TX, RB0 as input
 	TRISC =	0b10000000;		//hardware UART pins: RC7=RX, RC6=TX
	TRISD = 0b00000000;		//RD0 as output
	TRISE = 0b00000001;
	
	PORTB = 0b00000000;		//clear the PORTs
	PORTC = 0b00000000;
	PORTD = 0b00000000;
	PORTE = 0b00000000;
	
	//open software UART
	//* Please refer to MPLAB C18 C Compiler Libraries for description.
	OpenUART();

	//open hardware UART
	//* Please refer to MPLAB C18 C Compiler Libraries for description.
	//* Asynchronous mode, high speed: 129 = 20MHz / (9600 * 16) - 1
	OpenUSART( USART_TX_INT_OFF & USART_RX_INT_OFF & USART_ASYNCH_MODE & 
			   USART_EIGHT_BIT & USART_CONT_RX, 129 );
	
}


//-----------------------------------------------------------------------------------------
// * This is a delay function for user to use when the program need a delay.
// * This function can be call by type : delay(xxxx), where xxxx = 0->65535.
// * User can determine how long the program should delay for, the greater the value, the 
//	 longer the delay period.
//-----------------------------------------------------------------------------------------
void delay(unsigned long data)
{
	for( ;data>0;data-=1);
}


//-----------------------------------------------------------------------------------------
// * The delay functions for software UART.
// * User has to provide the appropriate delays so that the software UART is operating at 
//	 the desired baudrate. In this case, the baudrate is 9600, FOSC = 20MHz.
// * Change the i value to modify the delay period.
// * Please refer to MPLAB C18 C Compiler Libraries for the given formulas.
//-----------------------------------------------------------------------------------------
void DelayTXBitUART(void)
{
	//Delay for:(((2*20MHz)/(4*9600))+1)/2)-12 = 509 cycles
	unsigned i=25;
	for( ;i>0;i-=1);
}

void DelayRXHalfBitUART(void)
{
	//Delay for:(((2*20MHz)/(8*9600))+1)/2)-9 = 252 cycles
	unsigned i=12;
	for( ;i>0;i-=1);
}

void DelayRXBitUART(void)
{
	//Delay for:(((2*20MHz)/(4*9600))+1)/2)-14 = 507 cycles
	unsigned i=25;
	for( ;i>0;i-=1);
}


//-----------------------------------------------------------------------------------------
// * The UARTs control functions.
// * Please note that the functions given here are not all of the UART commands for SD02B.
// * Please refer to SD02B user's manual for more commands.
// * User can modify the functions to use in own application.
// * Functions for hardware UART: WriteUSART(), BusyUSART() -> control right motor driver
// * Functions for software UART: WriteUART()				-> control left motor driver
//-----------------------------------------------------------------------------------------
void On(void)
{
	WriteUART('O');				//On motors
	while(BusyUSART());
	WriteUSART('O');
}

void Off(void)
{
	WriteUART('F');				//Off motors
	while(BusyUSART());
	WriteUSART('F');
}

void Forward(unsigned char lspeed, unsigned char rspeed)
{
	WriteUART('>');				//Set direction
	while(BusyUSART());
	WriteUSART('<');
	
	WriteUART('G');				//Run motors
	while(BusyUSART());
	WriteUSART('G');

	WriteUART('S');				//Set speed
	WriteUART(lspeed);				
	while(BusyUSART());
	WriteUSART('S');
	while(BusyUSART());
	WriteUSART(rspeed);
}

void Backward(unsigned char lspeed, unsigned char rspeed)
{
	WriteUART('<');				//Set direction
	while(BusyUSART());
	WriteUSART('>');
	
	WriteUART('G');				//Run motors
	while(BusyUSART());
	WriteUSART('G');

	WriteUART('S');				//Set speed
	WriteUART(lspeed);				
	while(BusyUSART());
	WriteUSART('S');
	while(BusyUSART());
	WriteUSART(rspeed);
}

void Brake(void)
{
	WriteUART('B');				//Brake motors
	while(BusyUSART());
	WriteUSART('B');
}

void Encoder_R(unsigned int encoder)
{
	//Set a desired encoder value in 16 bits (0-65535)
	//Please refer to SD02B user's manual for the formula to calculate this value

	unsigned char encoder_H = (encoder>>8)&0x00FF;	//Formula to get the higher 8 bits
	unsigned char encoder_L	= encoder&0x00FF;		//Formula to get the lower 8 bits

	while(BusyUSART());
	WriteUSART('R');			//Reset encoder value first before start tracking.					
	while(BusyUSART());			//Please wait for the motors to stop.
	WriteUSART('T');
	while(BusyUSART());
	WriteUSART(encoder_H);
	while(BusyUSART());
	WriteUSART(encoder_L);
}

void Encoder_L(unsigned int encoder)
{
	//Set a desired encoder value in 16 bits (0-65535)
	//Please refer to SD02B user's manual for the formula to calculate this value

	unsigned char encoder_H = (encoder>>8)&0x00FF;	//Formula to get the higher 8 bits
	unsigned char encoder_L	= encoder&0x00FF;		//Formula to get the lower 8 bits

	WriteUART('R');				//Reset encoder value first before start tracking.					
	WriteUART('T');
	WriteUART(encoder_H);
	WriteUART(encoder_L);
}

void Accelerate_R(unsigned char initial_speed, unsigned char final_speed, unsigned char rate)
{
	while(BusyUSART());
	WriteUSART('A');			//Call acceleration
	while(BusyUSART());
	WriteUSART(initial_speed);	//Initial speed 	(value = 1 to 255)
	while(BusyUSART());
	WriteUSART(final_speed);	//Final speed 		(value = 1 to 255)
	while(BusyUSART());
	WriteUSART(rate);			//Acceleration rate (value = 1 to 255)
	while(BusyUSART());
	WriteUSART('G');			//Run the motor
}

void Accelerate_L(unsigned char initial_speed, unsigned char final_speed, unsigned char rate)
{
	WriteUART('A');				//Call acceleration
	WriteUART(initial_speed);	//Initial speed 	(value = 1 to 255)
	WriteUART(final_speed);		//Final speed 		(value = 1 to 255)
	WriteUART(rate);			//Acceleration rate (value = 1 to 255)
	WriteUART('G');				//Run the motor
}

void Microstep(unsigned char step)
{
	WriteUART('M');				//Set microstep for left motor
	WriteUART(step);			//step = 1(no micro stepping), 2, 5 or 10.

	while(BusyUSART());
	WriteUSART('M');			//Set microstep for right motor
	while(BusyUSART());
	WriteUSART(step);			//step = 1(no micro stepping), 2, 5 or 10.
}

unsigned int Check_Enc_R(void)
{
	unsigned int encoder_High;
	unsigned int encoder_Low;
	while (BusyUSART());
	WriteUSART('E');

	while (!DataRdyUSART());
	encoder_High=ReadUSART();
	encoder_High = ((encoder_High<<8) & 0xFF00); //rotate 8 bits to the left
                             				//make sure LSByte is '0'	
	while (!DataRdyUSART());
	encoder_Low=ReadUSART();
	encoder_Low = encoder_Low & 0x00FF;     //make sure MSByte is '0'
   	return (encoder_High ^ encoder_Low);   //XOR to make rvalue complete 16 bits
}

unsigned int Check_Enc_L(void)
{
	unsigned int encoder_High;
	unsigned int encoder_Low;
	WriteUART('E');

	encoder_High=ReadUART();
	encoder_High = ((encoder_High<<8) & 0xFF00); //rotate 8 bits to the left
                             			//make sure LSByte is '0'	
	encoder_Low=ReadUART();
	encoder_Low = encoder_Low & 0x00FF;     //make sure MSByte is '0'
   	return (encoder_High ^ encoder_Low);   //XOR to make rvalue complete 16 bits
}

//=========================================================================================
//	Interrupt vector
//=========================================================================================
#pragma	code InterruptVectorHigh = 0x08
void InterruptVectorHigh(void)
{
 	_asm
		goto ISRHigh		// jump to interrupt routine
	_endasm
}
#pragma code
#pragma	code InterruptVectorLow = 0x18
void InterruptVectorLow(void)
{
 	_asm
		goto ISRLow			// jump to interrupt routine
	_endasm
}
#pragma code

//=========================================================================================
//	Interupt Service Routine
//	This is the function reserved for interrupt service routine
//	User may need it in advance development of the program
//=========================================================================================
#pragma interrupt ISRHigh
void ISRHigh(void)
{

}

#pragma interrupt ISRLow
void ISRLow(void)
{

}

//=========================================================================================
//	End of Program
//=========================================================================================
