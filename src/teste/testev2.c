#include "mr32.h"
#include <stdlib.h>

#define STOP -1;
#define MOVING 0;
#define TURNING_LEFT 1;
#define TURNING_RIGHT 2;
#define BI_DIRECTION 3;
#define BI_DIRECTION_TWO 4;
#define BI_DIRECTION_THREE 5;

int stack[50] = {};
int stackSize = 0;
int isInverted = 0;
int isTurning = 0;
volatile static int turningTimer = 0;

// Velocidades
int speed = 40;
int adjust = 5;
int turningSpeed = 45;

int main(void)
{

	// Stack para guardar as turns

	// Configuracao dos leds
	TRISE = TRISE & 0xFFF0;
	int sensor;

	/*
	// Timer utilitario de 4 hz 
	T1CONbits.TCKPS = 2;		// 1:64 prescaler
	PR1 = PBCLK / (64 * (4 + 1));   // Fout = 20M / (64 * (4 + 1)) = 4 Hz
	TMR1 = 0;            		// Reset timer T2 count register
	T1CONbits.TON = 1;   		// Enable timer T2 (must be the last command of 

	IPC1bits.T1IP = 2;   // Interrupt priority (must be in range [1..6]) 
   	IEC0bits.T1IE = 0;   // Disable timer T2 interrupts 
   	IFS0bits.T1IF = 0;   // Reset timer T2 interrupt flag 

	*/

	// Inicializacao da pic
	initPIC32();
	closedLoopControl( false );
	setVel2(0, 0);	

	while(1) {

		while(!startButton()) {
		
		
			LATE = (LATE & 0xFFF0) | stack[stackSize - 1];
			delay(5000);
			LATE = (LATE & 0xFFF0) | stackSize;
			delay(5000);
			

		}
		
		do {	
			
			sensor = readLineSensors(0);
			LATE = (LATE & 0xFFF0) | (sensor & 0x0F);

			printInt(sensor, 10);

			if((sensor == 0x10) || (sensor == 0x08) || (sensor == 0x18) || (sensor == 0x1E)) sensor = 0x1C; // 10000 e 01000 e 11000 e 11110 para 11100;
			if((sensor == 0x01) || (sensor == 0x02) || (sensor == 0x03) || (sensor == 0x0F)) sensor = 0x07; // 00001 e 00010 e 00011 e 01111 para 00111;
		

			// Se o sensor detetar uma curva ou dead-end
			if(((sensor == 0x1C) || (sensor == 0x07) || (sensor == 0x00) || (sensor == 0x1F)) && !isTurning) {

				isTurning = 1;

				switch(sensor) {

					case 0x1C: // 11100
						setVel2(0, turningSpeed);
						stack[stackSize++] = TURNING_LEFT;
						break;
					case 0x07: // 00111
						setVel2(turningSpeed, 0);
						stack[stackSize++] = TURNING_RIGHT;
						break;
					case 0x00: // 00000
						setVel2(turningSpeed, -turningSpeed);
						isInverted = 1;
						break;
					case 0x1F: // 11111
						setVel2(turningSpeed, 0);	
						stack[stackSize++] = TURNING_LEFT;					
						break;

				}
			
			// Se estiver a ir em frente
			} else if((sensor == 0x04) || (sensor == 0x0E) || (sensor == 0x0C) || (sensor == 0x08) || (sensor == 0x06) || (sensor == 0x02)) {

				isTurning = 0;

				switch(sensor) {

					case 0x04: // 00100
					case 0x0E: // 01110
						setVel2(speed, speed);
						break;
					case 0x0C: // 01100
					case 0x08: // 01000
						setVel2(speed-adjust, speed+adjust);
						break;
					case 0x06: // 00110
					case 0x02: // 00010
						setVel2(speed+adjust, speed-adjust);
						break;

				}
			} 
					

		} while(!stopButton());

		setVel2(0,0);

	}	

}

void push(int direction) {

	stack[stackSize++] = direction;

}

int peek() {

	return stackSize != 0 ? stack[stackSize-1] : -1;

}

void pop() {

	stackSize--;

}

