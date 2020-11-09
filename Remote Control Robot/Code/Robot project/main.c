#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include "uart.h"
#include "i2c.h"
#include "mpu6050.h"
#include "mpu6050_reg.h"
#include "lcd.h"
#include "font.h"
#define F_CPU 16000000UL
#define BaudRate 9600
#define MYUBRR ( F_CPU / 16 / BaudRate ) -  1
#define RightMOTOR OCR0A
#define LeftMOTOR OCR0B

unsigned int joy = 0;		//create variable to store the received values from the joystick
uint16_t lengthSignal=0;	//lengthSignal to store the length of the echo signal from ultrasonic sensor
int count=0;				//count to check if echo signal from ultrasonic sensor is high or low
int counterAccel = 100;
int16_t buff[3];
double X, Y, Z;				//these variables save
int run = 1;				// this variable is the condition for running the loop in main. when run = 1 robot is running - when run = 0 robot stops
long xHighest, yHighest;
long prevX= 0, prevY = 0;
long changeX, changeY;
long checkCollisionX=22500;
long checkCollisionY=13500;


/************************************************************************
*Name: ISR(INT1_vect)                                                    *
*                                                                        *
*Purpose: stores the length of the echo signal and sends in to the remote*
*                                                                        *
*Input:	none															 *
*																		 *
*Output: none								                             *
*                                                                        *
*Calls: uart_putc(unsigned char data)								     *
************************************************************************/

ISR(INT1_vect){		                 //ISR for when there is a change in logic level (for ultrasonic)
	
	if (count == 0)		             //if count is 0, the echo signal is high
	{
		TCNT1 = 0;		             //set timer data register to 0
		count=1;	                 //set count to 1 (keeps track if echo signal is high or low)
	}
	else if(count == 1){	         //if count is 1, the echo signal is low
		lengthSignal=TCNT1;	         //store the length of the signal in lengthSignal
		count=0;					 //set count to 0 (keeps track if echo signal is high or low)
		if (lengthSignal>=255){		 //if length signal is over or equal to 255, set it to 255 (8-bit UDR0 register)
			lengthSignal=255;
		}
		
		//{
		if (lengthSignal<60){							//if (counerSignal=3) //only send when the counterSignal is less than to 50
			uart_putc(((unsigned char)lengthSignal)|1);		//sent lengthSignal to the remote as an unsigned char,
			// mask the value to distinguish between the accelerometer and the ultrasonic sensor data
		}
	}
}


/************************************************************************
*Name: initPWM()                                                        *
*                                                                       *
*Purpose: initialize the PWM								            *
*                                                                       *
*Input: none											                *
*                                                                       *
*Output: void								                            *
*                                                                       *
*Calls: none														    *
************************************************************************/

void initPWM()
{
	DDRD |= (1<< PORTD5); //set PORTD pin5 as an output (left motor)
	DDRD |= (1 << PORTD6);// set PORTD pin6 as an output (right motor)
	TCCR0A |= (1 << COM0A1);// set none-inverting mode for OCR0A
	TCCR0A |= (1 << COM0B1);// set none-inverting mode for OCR0B
	TCCR0A |= (1 << WGM01) | (1 << WGM00);// set fast PWM Mode
	TCCR0B |= (1 << CS01);// sets prescaler to 8 and starts PWM
}

/************************************************************************
*Name: initMotorDir()                                                   *
*                                                                       *
*Purpose: initialize the pins for the H-bridge for the motor direction  *
*                                                                       *
*Input: none											                *
*                                                                       *
*Output: void								                            *
*                                                                       *
*Calls: none														    *
************************************************************************/

void initMotorDir(){
	DDRD |= (1<< PORTD2); //set PORTD pin2 as an output (left motor)
	DDRD |= (1<< PORTD4); //set PORTD pin4 as an output	(left motor)
	DDRD |= (1<< PORTD7); //set PORTD pin7 as an output (right motor)
	DDRB |= (1<< PORTB0); //set PORTB pin0 as an output	(right motor)
}

/************************************************************************
*Name: motorDirection()                                                 *
*                                                                       *
*Purpose: sets the direction of the wheels depending on the input		*
*                                                                       *
*Input: decides in what directions wheels will spin		                *
*                                                                       *
*Output: void								                            *
*                                                                       *
*Calls:																    *
************************************************************************/
void motorDirection(int dir){
	if(dir==0){		//if dir equals 0, go backwards
		PORTD &= ~(1<<PORTD4);	//set D4 to low
		PORTD |= 1<<PORTD2;		//set D2 to high
		PORTD &= ~(1<<PORTD7);	//set D7 to low
		PORTB |= 1<<PORTB0;		//set B0 to high
	}
	else{	//if dir NOT 0, go forwards
		PORTD &= ~(1<<PORTD2);	//set D2 to low
		PORTD |= 1<<PORTD4;		//set D4 to high
		PORTB &= ~(1<<PORTB0);	//set B0 to low
		PORTD |= 1<<PORTD7;		//set D7 to high
	}
	
}

/************************************************************************
*Name: initTimer()	                                                    *
*                                                                       *
*Purpose: initialize the timer that counts the length of the echo signal*
*		  of the ultrasOnic sensor										*
*                                                                       *
*Input: none											                *
*                                                                       *
*Output: void								                            *
*                                                                       *
*Calls: none														    *
************************************************************************/
void initTimer(){
	TCCR1B = 0b00000101;			//set prescaler to 1024 for reduced overflow rates
	TCCR1A = 0b00000000;			//normal operation mode
	
}

/************************************************************************
*Name: initExtInt1()                                                    *
*                                                                       *
*Purpose: initialize Int1 to be triggered when echo signal has a logical*
*		  change														*
*                                                                       *
*Input: none											                *
*                                                                       *
*Output: void								                            *
*                                                                       *
*Calls: none														    *
************************************************************************/
void initExtInt1() {
	DDRD &= ~(1 << PORTD3);		//Set PIND3 as input (even if it is interrupt triggered)
	
	//The next two lines initialize interrupt code for external interrupt INT1
	EICRA |= (1 << ISC10);		//Set INT1 from the External Interrupt Control Register A to trigger ANY logic change
	EIMSK |= (1 << INT1);		//Set INT1 from the External Interrupt Mask Register to on (set pin 0 to 1)
	
	sei();						//Enable global interrupt
}

/************************************************************************
*Name: initTriggerPin()                                                 *
*                                                                       *
*Purpose: initialize PORTC pin2 as output to be used as a trigger pin	*
*													                    *
*Input: none											                *
*                                                                       *
*Output: void								                            *
*                                                                       *
*Calls: none														    *
************************************************************************/
void initTriggerPin(){
	DDRC |= 1<<PORTC2;				//set PINC2 to output (trig pin)
}



/************************************************************************
*Name: trigger()	                                                    *
*                                                                       *
*Purpose: triggers a signal for the ultrasonic sensor					*
*                                                                       *
*Input: none											                *
*                                                                       *
*Output: void								                            *
*                                                                       *
*Calls: none														    *
************************************************************************/
void trigger(){
	PORTC &= ~(1<<PORTC2);	//make sure pin C2 is low
	_delay_us(3);
	PORTC |= (1<<PORTC2);	//set pin C2 to high
	_delay_us(10);			//wait 10 microseconds
	PORTC &= ~(1<<PORTC2);	//set pin C2 to low
}




/************************************************************************************************************************
Name:																													*
*																														*
Purpose: This function  calculates the change in acceleration.															*
It takes the current acceleration in the x and y directions and compares them with the initialized threshold.           *
If its bigger or smaller than the negative of the threshold the vehicle comes to a stop.                                *
*																														*
input : void																											*
*																														*
return :    																											*
************************************************************************************************************************/
void calcChangeAcc(){
	
	changeX= X - prevX;   //saves the change in acceleration in (x) direction in a variable
	changeY= Y - prevY;	  //saves the change in acceleration in (y) direction in a variable
	
	if((changeX>checkCollisionX) || (changeX<(-1*checkCollisionX))){ //checks if the acceleration in X direction is bigger than the collision threshold
		OCR0A=0;											   // stops left motor
		OCR0B=0;											   // stops right motor
		uart_putc(0);										   // send to remote to show collision occurred
		//if condition is met robot stays in this loop indefinitely (until reset)
	}
	
	if((changeY>checkCollisionY) || (changeY<(-1*checkCollisionY))){
		OCR0A=0;											   // stops left motor
		OCR0B=0;											   // stops right motor
		uart_putc(0);										   // send to remote to show collision occurred
	}
	
	prevX = X;                                                     // stores the current acceleration in X direction in order for comparison against next reading
	prevY = Y;                                                     // stores the current acceleration in Y direction in order for comparison against next reading
}


/************************************************************************
*Name: collisionDetection()	                                            *
*                                                                       *
*Purpose: reads values from the mpu6050 accelerometer and stores them   *
*		  in an array to be read from later.							*
*
*Input: none											                *
*                                                                       *
*Output: void								                            *
*                                                                       *
*Calls: mpu6050_read_accel_ALL(buff)									*
************************************************************************/
void collisionDetection()
{
	mpu6050_read_accel_ALL(buff);
	X = buff[0];
	Y = buff[1];
	Z = buff[2];
}



/************************************************************************
*Name: main()		                                                    *
*                                                                       *
*Purpose: initialize functions, trigger signal for ultrasonic sensors,  *
make motors move according to received value. also reads accelerometer	*
*			values for collison detection.                              *
*Input: none											                *
*                                                                       *
*Output: int								                            *
*                                                                       *
*Calls: initTimer(), initExtInt1(), uart_init(), initPWM(),				*
initMotorDir(), initTriggerPin(), trigger(), uart_getc(),				*
motorDirection(), mpu6050_init(),collisionDetection(),					*					
************************************************************************/
int main(void)
{
	initTimer();
	initExtInt1();
	uart_init(UART_BAUD_SELECT(BaudRate,F_CPU));
	initPWM();
	initMotorDir();
	initTriggerPin();
	mpu6050_init();
	lcd_init(0xAF);										//init lcd and turn on
	sei();												//enable global interrupts
	int counter = 100;									//create variable to delay the trigger signal for the ultrasonic sensor	
	
	
	while (run)
	{
		counterAccel++;
		if(counterAccel==100){ //reads the accelerometer values every 100 iterations to give CPU more time to work on reading from joystick
			
			collisionDetection();  //this calls the function that reads the accelerometer values and stores them
			calcChangeAcc();       // calculates change in accelerations and sees if a collision has occurred
			
			counterAccel=0;
		}
		
		if (counter==100)							//to only trigger the signal every 100th time (delay)
		{
			trigger();								//trigger signal for ultrasonic sensor
			counter=0;								//set counter to 0
		}
		counter++;									//increment counter
		joy= uart_getc();							//store the received values from the joystick in joy
		if (joy==UART_NO_DATA)						//if joy is not receiving data
		{
			;										//do nothing
			}else{
			
			
			if (joy=='f')						    //if joystick is moving forward, drive forward
			{
				if (lengthSignal<=25)		//if distance is less or equal to 30, stop
				{
					RightMOTOR=0;			//stop right motor
					LeftMOTOR=0;			//stop left motor
					}else{						//else act accordingly
					motorDirection(1);           		//set motor direction to forward
					RightMOTOR=205;						//drive straight, right motor
					LeftMOTOR=205;				        //drive straight, left motor
					
				}
				}else if(joy=='b'){					//if joystick is moving backwards, drive backwards
				motorDirection(0);	                //set motor direction to backwards
				RightMOTOR=128;		                //drive straight, right motor
				LeftMOTOR=140;		                //drive straight, left motor
				
				
				}else if(joy=='r'){                 //if joystick is pressed forward right, drive turning right
				if (lengthSignal<=25)	//if distance is less or equal to 30, stop
				{
					RightMOTOR=0;		//stop right motor
					LeftMOTOR=0;		//stop left motor
					}else{				//else act accordingly
					motorDirection(1);					//set motor direction to forward
					RightMOTOR= 155;                    //right
					LeftMOTOR= 215;                     //left
				}
				}else if(joy=='l'){
				if (lengthSignal<=25)	//if distance is less or equal to 30, stop
				{
					RightMOTOR=0;		//stop right motor
					LeftMOTOR=0;		//stop left motor
					}else{				//else act accordingly
					motorDirection(1);					//set motor direction to forward
					RightMOTOR= 215;
					LeftMOTOR= 165;
				}
				}else if(joy=='s'){					//if joystick is in middle position, stop
				RightMOTOR=0;						//stop right motor
				LeftMOTOR=0;						//stop left motor
				
				
				}else if(joy=='w'){					//if joystick is in backwards left position
				motorDirection(0);					//set motor direction to backwards
				RightMOTOR=180;						//have the right motor running higher than the left motor
				LeftMOTOR=100;
				
				}else if(joy=='e'){					//if joystick is in backwards right position
				motorDirection(0);					//set motor direction to backwards
				RightMOTOR=100;
				LeftMOTOR=180;						//have the left motor running higher than the right motor
				}else{								//if error occurs drive slowly
				
				RightMOTOR=50;
				LeftMOTOR=50;
			}
		}
		
	}
	return 0;
}
