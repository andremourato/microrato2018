#include "mr32.h"

#define STOP -1;
#define MOVING 0;
#define TURNING_LEFT 1;
#define TURNING_RIGHT 2;
#define BI_DIRECTION 3;
#define BI_DIRECTION_TWO 4;
#define BI_DIRECTION_THREE 5;

static const int turningTime = 850; // hardcoded

volatile int millis = 0;
volatile int millisTurn = 0;

int stack[50] = {};
int stackSize = 0;
int isTurning = 0;
int sensorTurn = 0;

int isInversed = 0;
int mayHaveFinished = 0; //detects that it may have finished

int turnDetected = 0; // Para evitar o robo mudar de direcao no caso de os sensores detetarem a reta final quando nao deviam (dead end perto de meta, e ao rodar os sensores ficarem em contacto com a meta)

// Velocidades
int speed = 40;
int adjust = 10;
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

void pop();
int peek();
void push();
void waitingStart();

int main(void)
{

	// Configuracao dos leds
	TRISE = TRISE & 0xFFF0;
	int sensor;

	// Timer utilitario de 4 hz 
	T1CONbits.TCKPS = 3;		// 1:64 prescaler
	PR1 = PBCLK / 256 / 1000 - 1;   
	TMR1 = 0;            		// Reset timer T2 count register
	T1CONbits.TON = 1;   		// Enable timer T2 (must be the last command of 

	IPC1bits.T1IP = 2;   // Interrupt priority (must be in range [1..6]) 
   	IEC0bits.T1IE = 0;   // Disable timer T2 interrupts 
   	IFS0bits.T1IF = 0;   // Reset timer T2 interrupt flag 
	
	// Inicializacao da pic
	initPIC32();
	closedLoopControl( false );
	setVel2(0, 0);
	waitingStart();
	while(1) {
		
		do {
			sensor = readLineSensors(0);
			LATE = (LATE & 0xFFF0) | (sensor & 0x0F); //USED FOR DEBUG
			
			//If all sensors were enabled and it's still turning, means it found the end
			if(mayHaveFinished && millis > 700){ //FOUND THE GOAL!
				main();
			}
			if((sensor == 0x10) || (sensor == 0x18) || (sensor == 0x1E)) sensor = 0x1C; // 10000 e 11000 e 11110 para 11100;
			if((sensor == 0x01) || (sensor == 0x03) || (sensor == 0x0F)) sensor = 0x07; // 00001 e 00011 e 01111 para 00111;
		
			// ------------- Se o sensor detetar uma curva ----------
			if(((sensor == 0x07) || (sensor == 0x1C) || (sensor == 0x1F)) && !isTurning) {
				if(sensor == 0x1F){
					mayHaveFinished = 1;
					millis = 0;
				}
				if(isInversed) isInversed = 0;
				
				turnDetected = 1; // Vai ficar a 0 logo se houver caminho em frente

				if(sensor != 0x1C) {
					isTurning = 1;
					push(sensor);
					IEC0bits.T1IE = 1;
					setVel2(turningSpeed, - turningSpeed - adjust);
				}
					
				//LATE = (LATE & 0xFFF0) | isTurning << 2 | mayHaveFinished; //USED FOR DEBUG				
				
			// ---------- Se encontrar uma dead end -----------------
			} else if(sensor == 0x00 && !isTurning && !isInversed) {
				millis = 0;
				IEC0bits.T1IE = 1;
				setVel2(-reverseSpeed, reverseSpeed);
				isInversed = 1;
				isTurning = 1;

			// ----------- Se estiver a ir em frente ---------------
			// Serve como ponto de paragem de uma viragem ou inversao
			// Durante uma inversao e preciso ter cuidado com a meta, pois pode estar proximo
			// Do dead-end e os sensores centrais ao detetarem-ne fazem com que o robo saia de sitio 
			// turnDetected serve para reconhecer uma curva, e os millis para o caso acima da inversao
			// Sabe-se por tentativa que a inversao demora +' 730-850 ms, logo 
			} else if(((sensor == 0x04) || (sensor == 0x0E) || (sensor == 0x0C) || (sensor == 0x08) || (sensor == 0x06) || (sensor == 0x02)) 
													&& (turnDetected || millis > turningTime || !isTurning)){
				mayHaveFinished = 0;				
				// Quando ha so uma curva a esquerda mas como por default ele vai em frente contra um dead-end, ativa a flag
				// De inversao quando na realidade nao devia, pois so esta a virar
				if(isInversed && millis < 500)
					isInversed = 0;

				turnDetected = 0;
				isTurning = 0;
				IEC0bits.T1IE = 0;

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
		waitingStart();
	}	

}

void waitingStart(){
	while(!startButton()) {
		
		IEC0bits.T1IE = 0;
		/* THe LEDS blink repeateadly when the robot is waiting start */
		LATE = LATE & 0xFF00;
		delay(5000);
		LATE = (LATE & 0xFF00) | 0x00FF;
		delay(5000);
		/*LATE = (LATE & 0xFFF0) | turnDetected; //USED FOR DEBUG*/
		/*
		LATE = (LATE & 0xFFF0) | stack[stackSize - 1];
		delay(5000);
		LATE = (LATE & 0xFFF0) | stackSize;
		delay(5000);
		*/

	}
}

void _int_(4) isr_T1() {

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

