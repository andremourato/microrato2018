#include "mr32.h"

#define STOP -1;
#define MOVING 0;
#define TURNING_LEFT 1;
#define TURNING_RIGHT 2;
#define BI_DIRECTION 3;
#define BI_DIRECTION_TWO 4;
#define BI_DIRECTION_THREE 5;

#define LEFT_TURN 0x1C;
#define RIGHT_TURN 0x07;

static const int turningTime = 850; // hardcoded

volatile int millis = 0;
volatile int sensor;


int stack[50] = {};
int stackSize = 0;

int isTurning = 0;
int isInversing = 0;
int isInversed = 0;
int sideDetected = 0; // Para evitar o robo mudar de direcao no caso de os sensores detetarem a reta final quando nao deviam (dead end perto de meta, e ao rodar os sensores ficarem em contacto com a meta)

// Velocidades
int speed = 50;
int adjust = 20;
int turningSpeed = 40;
int reverseSpeed = 45;

/*
Para os valores
speed = 40;
adjust = 10;
turningSpeed = 40;
reverseSpeed = 45;
O robô consegue percorrer o labirinto todo em 1 minuto e 23 segundos.
*/

void configure(void);
void waitingStart(void);
void run(void);

int centerSensors(int);
int leftSensors(int);
int rightSensors(int);
int deadEndFound(int);
int biFound(int);

void reverseLeft(void);
void reverseRight(void);

// Funcoes para a stack
void pop(void);
int peek(void);
void push(int);

int main(void)
{

	configure();
	waitingStart();
	
}

void configure() {

	// Configuracao dos leds
	TRISE = TRISE & 0xFFF0;

	// Timer utilitario de 4 hz 
	T1CONbits.TCKPS = 3;		// 1:64 prescaler
	PR1 = PBCLK / 256 / 1000 - 1;   
	TMR1 = 0;            		// Reset timer T2 count register
	T1CONbits.TON = 1;   		// Enable timer T2 (must be the last command of 

	IPC1bits.T1IP = 2;   // Interrupt priority (must be in range [1..6]) 
   	IEC0bits.T1IE = 1;   // Disable timer T2 interrupts 
   	IFS0bits.T1IF = 0;   // Reset timer T2 interrupt flag 
	
	// Inicializacao da pic
	initPIC32();
	closedLoopControl( false );
	setVel2(0, 0);

}

void run() {

	while(1) {

		do {

			sensor = readLineSensors(0);

			// Se for detetada uma curva ou uma bifurcacao
			if(biFound(sensor) || ((leftSensors(sensor) || rightSensors(sensor)) && !isTurning)) {

				millis = 0;
				sideDetected = 1;

				// Se encontrou uma curva ou bifurcacao e estava em sentido inverso, deixa de estar
				// Nao acontece sempre assim, alterar depois
				if(isInversed) isInversed = 0;

				// Se nao for uma curva a esquerda, pois so vira a direita por default	
				if(!leftSensors(sensor)) { 
					isTurning = 1;
					reverseRight();
				}	

			// Se for detetado um dead end
			} else if(deadEndFound(sensor) && !isTurning) {
	
				millis = 0;
				isInversed = 1;
				isTurning = 1;
				reverseLeft();				

			} else if(((sideSensors(sensor) || centerSensors(sensor)) && !isTurning || sideDetected || millis >= turningTime)) {

				// Quando ha so uma curva a esquerda mas como por default ele vai em frente contra um dead-end, ativa a flag
				// De inversao quando na realidade nao devia, pois so esta a virar
				if(isInversed && millis < 500)
					isInversed = 0;
				
				if(isTurning && sideDetected && (millis > (turningTime * 0.66))) {
					setVel2(0,0);
					while(1);
				}

				isTurning = 0;
				sideDetected = 0;

				switch(sensor) {

					case 0x04: // 00100
					case 0x0E: // 01110
						setVel2(speed, speed);
						break;
					case 0x0C: // 01100
					case 0x08: // 01000
						setVel2(speed-adjust, speed+adjust);
						break;
					case 0x18: // 11000
					case 0x10: // 10000	
						
					case 0x06: // 00110
					case 0x02: // 00010
					//case 0x03: // 00011
					// case 0x01: // 00001
						setVel2(speed+adjust, speed-adjust);
						break;

				}

			// No caso de o robo estiver a sair da pista para a sua esquerda, vai detetar dead-end
			// E comecar a dar reverseLeft, o que iria inverter o seu sentido. Ao fazer reverseright
			// Quando o sensor 
			} else if(sensor == 0x01 && isTurning && millis < 100) {

				millis = 0;
				isTurning = 0;
				reverseRight();

			}

		} while(!stopButton());

		setVel2(0,0);
		waitingStart();
	}	

}

void waitingStart() { 

	IEC0bits.T1IE = 0;
	LATE = LATE & 0xFFF0;
	int i = 0;

	while(!startButton()) {
		sensor = readLineSensors(0);
		delay(100);
		if(++i >= 50) {
			LATE = LATE ^ 0x000F;
			i = 0;
		}

	}

	IEC0bits.T1IE = 1;
	run();


}

// Se os sensores centrais estao a 1 (00100 ou 01110 ou 01100 ou 00110)
int centerSensors(int sensor) {

	return (sensor == 0x04) || (sensor == 0x0E) || (sensor == 0x0C) || (sensor == 0x08) || (sensor == 0x06) || (sensor == 0x02);
}

int sideSensors(int sensor) {

	return (sensor == 0x01) || (sensor == 0x03) || (sensor == 0x10) || (sensor == 0x18);

}

// Se encontrou curva a esquerda (10000 ou 11000 ou 11100 ou 11110)
int leftSensors(int sensor) {

	return (sensor == 0x10) || (sensor == 0x18) || (sensor == 0x1C) || (sensor == 0x1E);

}

// Se encontrou curva a direita (00001 ou 00011 ou 00111 ou 01111)
int rightSensors(int sensor) {

	return (sensor == 0x01) || (sensor == 0x03) || (sensor == 0x07) || (sensor == 0x0F);

}

int deadEndFound(int sensor) {

	return (sensor == 0x00);

}

// Se encontrou uma bifurcacao (11111)
int biFound(int sensor) {
	
	return sensor == 0x1F;
	
}

void reverseRight() {

	setVel2(reverseSpeed, -reverseSpeed);

}


void reverseLeft() {

	setVel2(-reverseSpeed, reverseSpeed);

}

void _int_(4) isr_T1() {
			
	LATE = (LATE & 0xFFF0) | (sensor & 0x0F); //USED FOR DEBUG
	millis++;	
	printf("%d\n", millis);
	IFS0bits.T1IF = 0;

}

void push(int sensor) {

	stack[stackSize++] = sensor;

}

int peek() {

	return stackSize != 0 ? stack[stackSize-1] : -1;

}

void pop() {

	stackSize--;

}

