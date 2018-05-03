#include "mr32.h"

#define STOP -1;
#define MOVING 0;
#define TURNING_LEFT 1;
#define TURNING_RIGHT 2;
#define BI_DIRECTION 3;
#define BI_DIRECTION_TWO 4;
#define BI_DIRECTION_THREE 5;

static const int turningTime = 700; // hardcoded
static int delayDeadEnd = 150;
static int delayTurn = 1250;

volatile int millis = 0; // Serve para medir o tempo entre cada branch
volatile int millisUtil = 0; 
volatile int sensor;
int lastSensor;

int stack[50] = {};
int stackSize = 0;

int straight = 0;
int isTurning = 0;
int isInversed = 0;
int sideDetected = 0;
int possibleDeadEnd = 0;

// Velocidades
const int startSpeed = 40;
const int maxSpeed = 60;
int speed = 40;
const int startAdjust = 4;
const int maxAdjust = 8;
int adjust = 5;
int turningSpeed = 45;
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

void adjustPosition(void);
void updateSpeed(int);

int centerFound();
int rightFound();
int leftFound();
int deadEnd();
int biFound();

void reverseLeft(void);
void reverseRight(void);

// Funcoes para a stack
void pop(void);
int peek(void);
void push(int);

int main(void)
{

	configure();

	while(1) {

		waitingStart();

		do {

			sensor = readLineSensors(0);

			// Se os sensores ficarem todos a 0, ativa uma flag que se continuar a 1 passado
			// x tempo, (valor x hardcoded), ha uma confirmacao de que é um dead end	
			// Para se por acaso sair um pouco enquanto vai em linha reta e os sensores 
			// ficarem a 0000 nao comece a inverter sentido
	
			if(deadEnd()) {
				if(!isTurning && !possibleDeadEnd) {
					if((speed -= 5) < startSpeed) speed = startSpeed;		
					millisUtil = 0;
					possibleDeadEnd = 1;
				} else if(possibleDeadEnd && (millisUtil > delayDeadEnd)) {
					adjust = startAdjust;					
					speed = startSpeed;
					millis = 0;
					possibleDeadEnd = 0;
					isTurning = 1;
					if(sideDetected == 0) sideDetected = 4;
					reverseLeft();				
				}
			} else {
				possibleDeadEnd = 0;
			}
			
			// Se uma bifurcacao for encontrada ou uma curva for encontrada e ja nao estivera virar
			// Millis > 200 para nao detetar uma curva por engano ao estar a acabar uma (com os sensores 00011 ou 11000)	
			if((biFound() || ((leftFound() || rightFound()) && !isTurning)) && millis > 200) {

				adjust = startAdjust;
				speed = startSpeed;
				setVel2(speed, speed);

				if(rightFound()) sideDetected = 1;
				else if(leftFound()) sideDetected = 2;
				else sideDetected = 3;

				if(sideDetected != 2) { 
					isTurning = 1;
					delay(delayTurn);
					millis = 0;
					reverseRight();
				} else {
					millisUtil = 0;
				}
	
			// Se os sensores do centro estiverem ativos e o robo estiver a virar
			} else if((((sideDetected == 1) || (sideDetected == 3)) && (sensor == 0x01) && isTurning) ||
				   ((sideDetected == 2) && (sensor == 0x10) && isTurning) || 
			           ((sideDetected == 4) && (sensor == 0x10) && (millis >= turningTime) && isTurning) ||
				  (((sideDetected == 0) || (sideDetected == 2)) && !isTurning)) {

				updateSpeed(sensor);

				// No caso de ser curva a esquerda nao retirar o sideDetected, e tambem retira o sideDetected caso
				// Tambem possa ir em frente, onde e suposto manter o sideDetected a 0
				if(sideDetected == 2 && (millisUtil > 250)) sideDetected = 0;
				else if(sideDetected != 2) sideDetected = 0;
				if(isTurning) {
					millis = 0;
					isTurning = 0;
				}
			
			}

			lastSensor = sensor;

		} while(!stopButton());


		setVel2(0,0);
		waitingStart();
	
	}

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

void updateSpeed(int sensor) {

	switch(sensor) {

		case 0x04: // 00100
		case 0x0E: // 01110
			setVel2(speed, speed);
			break;
		case 0x0C: // 01100
			setVel2(speed-(adjust / 2), speed+(adjust / 2));
			break;
		case 0x08: // 01000
			setVel2(speed-adjust, speed+adjust);
			break;
		case 0x06: // 00110
			setVel2(speed+(adjust / 2), speed-(adjust / 2));
			break;
		case 0x02: // 00010
			setVel2(speed+adjust, speed-adjust);
			break;
		case 0x10: // 10000	
			if(isTurning) 
				setVel2(-reverseSpeed / 4, reverseSpeed / 4);
			else 
				setVel2(-10, speed);
			break;
		case 0x01: // 00001
			if(isTurning) 
				setVel2(reverseSpeed / 4, -reverseSpeed / 4);
			else 
				setVel2(speed, -10);
			break;

	}

}

void waitingStart() { 

	IEC0bits.T1IE = 0;
	int i = 0;

	while(!startButton()) {
		sensor = readLineSensors(0);
		delay(100);
		if(++i >= 50) {
			leds(sideDetected);
 			i = 0;
		}

	}

	IEC0bits.T1IE = 1;

}

// Se o robo encontrar uma linha preto por debaixo dele (00100 ou 01110 ou 01100 ou 00110)
int centerFound() {
	
	return (sensor == 0x04) || (sensor == 0x0E) || (sensor == 0x0C) || (sensor == 0x06);

}

// Se o robo encontrar curva a direita (00011 ou 00111 ou 01111)
int rightFound() {

	return (sensor == 0x03) || (sensor == 0x07) || (sensor == 0x0F);

}

// Se o robo encontrar curva a esquerda (11000 ou 11100 ou 11110)
int leftFound() {

	return (sensor == 0x18) || (sensor == 0x1C) || (sensor == 0x1E);

}

// Se o robo encotrar uma bifurcacao ou a meta de chegada (11111)
int biFound() {

	return (sensor == 0x1F);

}

// Se o robo encontar uma dead end (ou quando há uma curva a esquerda)
int deadEnd() {

	return (sensor == 0x00);

}

void reverseRight() {

	setVel2(reverseSpeed, -reverseSpeed);

}


void reverseLeft() {

	setVel2(-reverseSpeed, reverseSpeed);

}

void _int_(4) isr_T1() {

	static int count = 0;
			
	leds(sensor); //USED FOR DEBUG

	millis++;	
	millisUtil++;
	printf("%d\n", millis);

	count++;
	if((count % 50) == 0)
		if(speed < maxSpeed) speed++; // Speed aumenta 1 a cada 50ms
	if(count >= 500) {
		if(adjust < maxAdjust) adjust++;	
		count = 0;
	}

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

