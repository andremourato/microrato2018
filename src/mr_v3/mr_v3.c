#include "mr32.h"

#define ERROR_LEVEL_1 2 //Melhor: 2
#define ERROR_LEVEL_2 3 //Melhor: 3
#define RIGHT_TURN_CONSTANT 5 //Valor máximo do contador. Vai depender da velocidade
#define DEAD_END_CONSTANT 5 //É necessário calibrar estes valores

//**************************** Funções auxiliares Declarações ****************************
int leftDetected(); // 0 -> bit da esquerda está a 0 | 1 -> bit da esquerda está a 1
int rightDetected(); // 0 -> bit da direita está a 0 | 1 -> bit da direita está a 1
int deadEndDetected(); //0 -> quando existe pelo menos 1 bit ON | 1 -> quando todos os bits estao OFF
double getRealSpeed();
void stopRobot();
void resetAllVariables();
void adjust();
void turnRight();
void turnLeft();
void invertDirection();
void updateSensorHistory();
//**************************** Variáveis do sistema ****************************
volatile int millis = 0;

//PID
/* Para calibrar Kp, Ki e Kd: calibra-se sempre de cima para baixo. Primeiro calibra-se
o Kp com o Ki = 0 e o Kd = 0. Testem várias vezes com valores diferentes.
Quando ele estiver bom, alterem o Ki e assim sucessivamente*/
/*Melhor conjunto de constantes encontrado: Kp=1,Kd=26,ERROR_LEVEL_1=2,ERROR_LEVEL_2=3*/
//Proporionalidade
double Kp = 1; //Contante de proporcionalidade. A melhor foi Kp = 3
//Derivada
double errorTable[] = {-ERROR_LEVEL_2, -ERROR_LEVEL_1, 0, ERROR_LEVEL_1, ERROR_LEVEL_2};
double Kd = 26; //Melhor: 26
int D;
double prevError;

//Constantes
double ROBOT_DIAMETER = 0.120; //Em m
double ROBOT_RADIUS;
double MAX_SPEED =  0.15; //Em m/s

// Velocidades
int speed = 70; //Percentagem da velocidade máxima do motor. Velocidade em linha reta
int turningSpeed = 40; //Velocidade a virar

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
int rightDetected(){ return sensor & 0x1; }
int leftDetected(){ return sensor >> 4; }
int deadEndDetected(){ return sensor == 0; }
double getRealSpeed(){ return speed*MAX_SPEED/100.0; } //a variavel speed é apenas a percentagem da velocidade máxima
int detectedLineAhead(){return sensor == 0b00100 || sensor == 0b01100 || sensor == 0b01000 ||
							   sensor == 0b00110 || sensor == 0b00010;}
void stopRobot(){ setVel2(0,0); }
void resetAllVariables(){ stopRobot(); }
//Calcula o PID
void adjust(){
	int middleSensors = (sensor & 0x0E) >> 1; //Gets the values of the 3 middle sensors
	int error = 0;
	switch(middleSensors){
		case 0b100: error = errorTable[0]; break;
		case 0b110: error = errorTable[1]; break;
		case 0b010: error = errorTable[2]; break;
		case 0b011: error = errorTable[3]; break;
		case 0b001: error = errorTable[4]; break;
	}
	D = error-prevError;
	int leftSpeed  = speed + (int)(Kp*(double)error) + (int)((double)D*Kd);
	int rightSpeed = speed - (int)(Kp*(double)error) - (int)((double)D*Kd);
	setVel2(leftSpeed,rightSpeed);
	prevError = error;
	//printf("Error=%d | %d | %d\n",error,leftSpeed,rightSpeed);
}
void turnRight(){
	led(2,1);
	setVel2(turningSpeed,-turningSpeed);
	while(!detectedLineAhead()){ sensor = readLineSensors(0); }
	led(2,0);
}
void turnLeft(){
	led(3,1);
	setVel2(-turningSpeed,turningSpeed);
	while(!detectedLineAhead()){ sensor = readLineSensors(0); }
	led(3,0);
}
void invertDirection(){
	led(4,1);
	setVel2(-turningSpeed,turningSpeed); //Vira à esquerda até encontrar a linha de novo
	while(!detectedLineAhead()){ sensor = readLineSensors(0); }
	led(4,0);
}
void updateSensorHistory(){
	if(rightDetected()) rightCounter += 1;
	else rightCounter -= 1;
	rightCounter = rightCounter <= 0 ? 0 : rightCounter;
	if(leftDetected()) leftCounter += 1;
	else leftCounter -= 1;
	leftCounter = leftCounter <= 0 ? 0 : leftCounter;
	if(deadEndDetected()) deadEndCounter += 1;
	else deadEndCounter -= 1;
	deadEndCounter = deadEndCounter <= 0 ? 0 : deadEndCounter;
	//printf("right = %d\n",rightCounter);
}

//**************************** Algoritmos para percorrer o labirinto ****************************
/* Algoritmo para preencher a stack. Vai virar sempre à direita */
void findBestPath(){
	int finished = 0; //variavel que dará por terminado o jogo. Fica a 1 quando encontrou o objetivo
	while(!finished && !stopButton()) {
		sensor = readLineSensors(0);
		updateSensorHistory(); //Faz o update dos contadores
		if(rightCounter >= RIGHT_TURN_CONSTANT) //Se estiver no centro, roda sobre si para a direita
			turnRight();
		else if(deadEndCounter >= DEAD_END_CONSTANT) //Se estiver no centro, inverte a marcha
			invertDirection();
		else	//Ajusta a rota caso esteja em linha reta
			adjust();
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
   	IEC0bits.T1IE = 0;   // Disable timer T2 interrupts 
   	IFS0bits.T1IF = 0;   // Reset timer T2 interrupt flag
   	IEC0bits.T1IE = 1; 	 //Enables interrupts
}

/* Usada para inicializar variaveis antes de começar */
void initializeVariables(){

}

/*Atualiza o temporizador*/
void _int_(4) isr_T1() {
	millis++;
	IFS0bits.T1IF = 0;

}
