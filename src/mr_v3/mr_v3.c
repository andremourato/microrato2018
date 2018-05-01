#include "mr32.h"

//**************************** Funções auxiliares Declarações ****************************
double getRealSpeed();
void stopRobot();
void resetAllVariables();
void adjustToTheLeft();
void adjustToTheRight();
void goForward();
void turnRight();
void turnLeft();
void invertDirection();
//**************************** Variáveis do sistema ****************************
volatile int millis = 0;
//Constantes
double ROBOT_DIAMETER = 0.120; //Em m
double ROBOT_RADIUS;
double MAX_SPEED =  0.15; //Em m/s
// Velocidades
int speed = 45; //Percentagem da velocidade máxima do motor. Velocidade em linha reta
int adjust = 8; //Velocidade de ajuste. Vai ser somado a uma roda e subtraido à outra
int turningSpeed = 40; //Velocidade a virar
volatile int sensor;
//Delays
int TIME_TO_CENTER; //Em ms. Tempo que demora a percorrer a distancia entre os sensores e o centro do robot
int TIME_TO_ROTATE_90;
//Informação sobre o estado do jogo
//Número de tentativas. Incrementado no final de cada tentativa
static int numTries = 0; //Não deve ser feito o reset desta variável

//**************************** Funções auxiliares (sensores,etc.) ****************************
double getRealSpeed(){ return speed*MAX_SPEED/100.0; } //a variavel speed é apenas a percentagem da velocidade máxima
int detectedLineAhead(){return sensor == 0b00100 || sensor == 0b01110 || sensor == 0b01100 ||
							   sensor == 0b01000 || sensor == 0b00110 || sensor == 0b00010;}
void stopRobot(){ setVel2(0,0); }
void resetAllVariables(){ stopRobot(); }
void adjustToTheLeft(){ setVel2(speed-adjust,speed+adjust); }
void adjustToTheRight(){ setVel2(speed+adjust,speed-adjust); }
void goForward(){ setVel2(speed,speed); }
void turnRight(){
	setVel2(turningSpeed,-turningSpeed);
	delay(TIME_TO_ROTATE_90);
	while(!detectedLineAhead()){ sensor = readLineSensors(0); }
}
void turnLeft(){
	setVel2(-turningSpeed,turningSpeed);
	delay(TIME_TO_ROTATE_90);
	while(!detectedLineAhead()){ sensor = readLineSensors(0); }
}
void invertDirection(){
	setVel2(-turningSpeed,turningSpeed); //Vira à esquerda até encontrar a linha de novo
	delay(TIME_TO_ROTATE_90);
	while(!detectedLineAhead()){ sensor = readLineSensors(0); }
}

//**************************** Algoritmos para percorrer o labirinto ****************************
/* Algoritmo para preencher a stack. Vai virar sempre à direita */
void findBestPath(){
	int finished = 0;
	while(!finished && !stopButton()) {
		sensor = readLineSensors(0);
		switch(sensor){
				//Deteta DEAD-END
				case 0b00000:
					invertDirection();
					break;
				//Caso detete biforcação ou meta
				case 0b11111:
					
				//Deteta curva à direita
				case 0b00001:
				case 0b00011:
				case 0b00111:
				case 0b01111:
					turnRight();
					break;
				case 0b00100:
				case 0b01110: //Sensores da frente ativos
					goForward();
					break;
				case 0b01100:
				case 0b01000: //Está a fugir para a direita
					adjustToTheLeft();
					break;
				case 0b00110:
				case 0b00010: //Está a fugir para a esquerda
					adjustToTheRight();
					break;
				//Deteta curva à esquerda
				//Neste algoritmo o robot vai ignorar
				//as curvas à esquerda
				case 0b10000:
				case 0b11000:
				case 0b11100:
				case 0b11110:
					//turnLeft();
					break;
		}
	}
}

//**************************** Funções para controlar o estado do robot ****************************
//Estas funções tem o objetivo de: inicializar o robot (timer,variaveis,etc.),
//decidir se o robot está à espera do botão start e determinar se o robot já conhece o caminho mais rápido
//ou se ainda vai percorrer o labirinto pela primeira vez

void run();
void waitingStart();
void configureRobot();
void configureTimer();
void calculateVariables(); //Calcula os tempos de acordo com a velocidade

int main(void)
{

	configureRobot(); //Configura o robot
	//waitingStart();
	return 0;
}

/* Vai determinar se o robot já sabe o caminho mais curto ou não */
void run(){
	//Se for a primeira tentativa, ainda tem que encontrar o caminho
	//if(numTries==0)
		findBestPath(); //Preenche a stack
	//else //Caso contrario, vai percorrer o caminho mais curto
		//chooseBestPath(); //Será descomentado mais tarde. Segue a stack
	numTries += 1;
	waitingStart();
}

/*Enquanto o botao start nao for premido, fica à espera*/
void waitingStart(){
	resetAllVariables();
	while(!startButton()); //Enquanto o botão start nao for premido
	run(); //Executa uma nova tentativa
}

/*Configurações do robot*/
void configureRobot(){
	//Calcula variaveis: tempos,etc.
	calculateVariables();
	// Configuracao dos leds
	TRISE = TRISE & 0xFFF0;
	//Configura o timer
	configureTimer();
	// Inicializacao da pic
	initPIC32();
	closedLoopControl(true); //Ativa a estabilização por PID
	stopRobot();
}

/*Configura o temporizador*/
void configureTimer(){
	// Timer utilitario de 4 hz 
	T1CONbits.TCKPS = 3;		// 1:64 prescaler
	PR1 = PBCLK / 256 / 1000 - 1;   
	TMR1 = 0;            		// Reset timer T2 count register
	T1CONbits.TON = 1;   		// Enable timer T2 (must be the last command of
	IPC1bits.T1IP = 2;   // Interrupt priority (must be in range [1..6]) 
   	IEC0bits.T1IE = 0;   // Disable timer T2 interrupts 
   	IFS0bits.T1IF = 0;   // Reset timer T2 interrupt flag
   	IEC0bits.T1IE = 1; 	 //Enables interrupts
}

void calculateVariables(){
	TIME_TO_ROTATE_90 = 450; //Em ms. Por tentativa erro. Calcular, mais tarde
	ROBOT_RADIUS = ROBOT_DIAMETER/2.0;
	TIME_TO_CENTER = 1000.0 * (ROBOT_RADIUS - 0.01) / getRealSpeed();
}

/*Atualiza o temporizador*/
void _int_(4) isr_T1() {
	millis++;
	IFS0bits.T1IF = 0;

}



