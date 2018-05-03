#include "mr32.h"

/*
* MELHORES RATIOS PARA VELOCIDADES:
------------------------------------------------------------
speed = 40
turningSpeed = 40
TURNING_CONSTANT=RIGHT_TURN_CONSTANT=DEAD_END_CONSTANT=16
------------------------------------------------------------
*/
#define ERROR_LEVEL_1 1 //Melhor: 2
#define ERROR_LEVEL_2 8.5 //Melhor: 3
#define TURNING_CONSTANT 2
#define RIGHT_TURN_CONSTANT TURNING_CONSTANT //Valor máximo do contador. Vai depender da velocidade
#define DEAD_END_CONSTANT TURNING_CONSTANT //É necessário calibrar estes valores

//**************************** Funções auxiliares Declarações ****************************
int sensorGet(int i); //Devolve o valor do sensor de indice i. 0->sensor da esquerda, 1->sensor central da esquerda, ..., 4->sensor da direita
int leftDetected(); // 0 -> bit da esquerda está a 0 | 1 -> bit da esquerda está a 1
int rightDetected(); // 0 -> bit da direita está a 0 | 1 -> bit da direita está a 1
int deadEndDetected(); //0 -> quando existe pelo menos 1 bit ON | 1 -> quando todos os bits estao OFF
int detectedLineAhead();
double getRealSpeed();
void stopRobot();
void resetAllVariables();
void adjust();
void turnRight();
void turnLeft();
void invertDirection();
//**************************** Variáveis do sistema ****************************
volatile int millis = 0;

//PID
/* Para calibrar Kp, Ki e Kd: calibra-se sempre de cima para baixo. Primeiro calibra-se
o Kp com o Ki = 0 e o Kd = 0. Testem várias vezes com valores diferentes.
Quando ele estiver bom, alterem o Ki e assim sucessivamente*/
/*Melhor conjunto de constantes encontrado: Kp=1,Kd=26,ERROR_LEVEL_1=2,ERROR_LEVEL_2=3*/
// Para speed = 70 : Kp=1,Kd=26,ERROR_LEVEL_1=2,ERROR_LEVEL_2=3
// Para speed = 50 : Kp=1.5,Kd=10,ERROR_LEVEL_1=2,ERROR_LEVEL_2=3
//Proporionalidade
double Kp = 2; //Contante de proporcionalidade. A melhor foi Kp = 3
//Derivada
double errorTable[] = {-ERROR_LEVEL_2, -ERROR_LEVEL_1, 0, ERROR_LEVEL_1, ERROR_LEVEL_2};
double Kd = 28; //Melhor: 26
int D;
double prevError;

//Constantes
double ROBOT_DIAMETER = 0.120; //Em m
double ROBOT_RADIUS;
double MAX_SPEED =  0.15; //Em m/s

// Velocidades
int speed = 50; //ercentagem da velocidade máxima do motor. Velocidade em linha reta
int turningSpeed = 40; //Velocidade a virar.

//Histórico de medidas
volatile int sensor;
//Contadores. São limitados inferiormente a 0, i.e., counter >= 0
int rightCounter = 0;
int leftCounter = 0;
int deadEndCounter = 0;

//Informação sobre o estado do jogo
//Número de tentativas. Incrementado no final de cada tentativa
static int numTries = 0; //Não deve ser feito o reset desta variável

//**************************** Funções auxiliares (sensores,etc.) ****************************
/* 0 <= i <= 4*/
int sensorGet(int i){
	return (sensor >> (4-i)) & 0x01;
}
void readSensors(){
	sensor = readLineSensors(0);
}
int rightDetected(){ return sensorGet(4); }
int leftDetected(){ return sensorGet(0); }
int deadEndDetected(){ return sensor == 0; }
double getRealSpeed(){ return speed*MAX_SPEED/100.0; } //a variavel speed é apenas a percentagem da velocidade máxima
int detectedLineAhead(){ return (sensorGet(1) || sensorGet(2) || sensorGet(3)) && (!sensorGet(0) && !sensorGet(4)); }
void stopRobot(){ setVel2(0,0); }
void resetAllVariables(){ stopRobot(); }
//Calcula o PID
void adjust(){
	int middleSensors = (sensor & 0x0E) >> 1; //Gets the values of the 3 middle sensors
	int error = 0;
	if(sensorGet(0)){
		if(sensorGet(3)) error = errorTable[1];
		else error = errorTable[3];
	}else{
		switch(middleSensors){
			case 0b100: error = errorTable[0]; break;
			case 0b110: error = errorTable[1]; break;
			case 0b010: error = errorTable[2]; break;
			case 0b011: error = errorTable[3]; break;
			case 0b001: error = errorTable[4]; break;
		}
	}
	D = error-prevError;
	int leftSpeed  = speed + (int)(Kp*(double)error) + (int)((double)D*Kd);
	int rightSpeed = speed - (int)(Kp*(double)error) - (int)((double)D*Kd);
	setVel2(leftSpeed,rightSpeed);
	prevError = error;
	printf("Error=%d | %d | %d\n",error,leftSpeed,rightSpeed);
}
void turnRight(){
	led(2,1);
	setVel2(turningSpeed,-turningSpeed);
	while(!sensorGet(4)) {readSensors();}	
	while(!detectedLineAhead()){ readSensors(); }
	led(2,0);
}
void turnLeft(){
	led(3,1);
	setVel2(-turningSpeed,turningSpeed);
	while(!sensorGet(0)) {readSensors();}
	while(!detectedLineAhead()){ readSensors(); }
	led(3,0);
}
void invertDirection(){
	led(4,1);
	setVel2(-turningSpeed,turningSpeed); //Vira à esquerda até encontrar a linha de novo
	while(!sensorGet(0)) {readSensors();}
	while(!detectedLineAhead()){ readSensors(); }
	led(4,0);
}

//**************************** Algoritmos para percorrer o labirinto ****************************
/* Algoritmo para preencher a stack. Vai virar sempre à direita */
void findBestPath(){

	const static int countAim = 6;
	static int countR, countL, countC = 0;
	static int turnDetected = 0;
	
	while(!stopButton()) {

		readSensors();
		adjust();
		//printInt(sensor, 2 | 5 << 16);
		//printf("\n");

		if(!turnDetected && ((sensor & 0x11) != 0)) turnDetected = 1;

		if(turnDetected) {

			if((sensor & 0x11) == 0) {
			
				// FUNCAO PARA DETETAR O QUE E A FAZER
				if((countR >= 15) && (countL >= 15)) {
					setVel2(0,0);
					while(1);
				} else if((countR >= countAim) && (countL >= countAim)) {
					turnRight();
					// CONDICOES STACK
				} else if((countR >= countAim) && (countL < countAim)) {
					turnRight();
					// CONDICOES STACK
				} else if((countL >= countAim) && (countR < countAim)) {
					if((sensor & 0x0E) == 0) turnLeft(); 
					// CONDICOES STACK
				}

				countL = 0; countR = 0; countC = 0;
				turnDetected = 0;

			} else {
				if((sensor & 0x10) != 0) countL++;
				if((sensor & 0x01) != 0) countR++;
				if((sensor & 0xE0) != 0) countC++;
			}

		} else if(sensor == 0) {
			invertDirection();
		}

	}
	
	/*
	int finished = 0; //variavel que dará por terminado o jogo. Fica a 1 quando encontrou o objetivo
	while(!finished && !stopButton()) {
		readSensors();
		
		updateSensorCounter(); //Faz o update dos contadores
		if(rightCounter >= RIGHT_TURN_CONSTANT) //Se estiver no centro, roda sobre si para a direita
			turnRight();
		else if(deadEndCounter >= DEAD_END_CONSTANT) //Se estiver no centro, inverte a marcha
			invertDirection();
		else	//Ajusta a rota caso esteja em linha reta
		
			adjust();
	}
	*/
}

//**************************** Funções para controlar o estado do robot ****************************
//Estas funções tem o objetivo de: inicializar o robot (timer,variaveis,etc.),
//decidir se o robot está à espera do botão start e determinar se o robot já conhece o caminho mais rápido
//ou se ainda vai percorrer o labirinto pela primeira vez

void run();
void waitingStart();
void configureRobot();
void configureTimer();
void initializeVariables(); //Calcula os tempos de acordo com a velocidade

int main(void)
{

	configureRobot(); //Configura o robot
	waitingStart();
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
	initializeVariables();
	// Configuracao dos leds
	TRISE = TRISE & 0xFFF0;
	//Configura o timer
	configureTimer();
	// Inicializacao da pic
	initPIC32();
	closedLoopControl(false); //Ativa a estabilização por PID
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
   	IFS0bits.T1IF = 0;   // Reset timer T2 interrupt flag
   	IEC0bits.T1IE = 1;   //Enables interrupts
}

/* Usada para inicializar variaveis antes de começar */
void initializeVariables(){

}

/*Atualiza o temporizador*/
void _int_(4) isr_T1() {
	millis++;
	IFS0bits.T1IF = 0;

}
