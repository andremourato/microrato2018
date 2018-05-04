#include "mr32.h"

/*
* MELHORES RATIOS PARA VELOCIDADES (Fazem o percurso todo):
------------------------------------------------------------
speed = 50
Kp = 2
Kd = 28
ERROR_LEVEL_1 = 1
ERROR_LEVEL_2 = 8.5
------------------------------------------------------------
*/
#define ERROR_LEVEL_1 1
#define ERROR_LEVEL_2 3
#define TURNING_CONSTANT 2
#define GOAL_CONSTANT 10
#define RIGHT_TURN_CONSTANT TURNING_CONSTANT //Valor máximo do contador. Vai depender da velocidade
#define DEAD_END_CONSTANT TURNING_CONSTANT //É necessário calibrar estes valores
//Identificadores
#define R 1 //Curva à direita
#define L 2 //Curva à esquerda
#define S 3 //Segue em frente

//**************************** Funções auxiliares Declarações ****************************
int sensorGet(int i); //Devolve o valor do sensor de indice i. 0->sensor da esquerda, 1->sensor central da esquerda, ..., 4->sensor da direita
int leftDetected(); // 0 -> bit da esquerda está a 0 | 1 -> bit da esquerda está a 1
int rightDetected(); // 0 -> bit da direita está a 0 | 1 -> bit da direita está a 1
int deadEndDetected(); //0 -> quando existe pelo menos 1 bit ON | 1 -> quando todos os bits estao OFF
int detectedLineAhead();
int turnDetected();
double getRealSpeed();
void stopRobot();
void resetAllVariables();
void adjust();
void turnRight();
void turnLeft();
void invertDirection();
void waitingStart();
void fillTheStack(int countC, int countR, int countL, int countAim, int isInverted);
void chooseBestPath();
/**************************** Stack related methods ***************************************/
//Identificadores: { R, L, S }
//ID Stack - stack que guarda os identificadores
int idStackTopIsEmpty();
int idStackIsFull();
int idStackPeek();
int idStackPop();
void idStackPush(int data);
void idStackPrint(); //USADO PARA DEBUG
//STACK DO NUMERO DE RAMOS POR INVESTIGAR
int branchStackIsEmpty();
int branchStackIsFull();
int branchStackPeek();
int branchStackPop();
void branchStackPush(int data);
void branchStackPrint(); //USADO PARA DEBUG
//**************************** Variáveis do sistema ****************************
volatile int millis = 0;

//PID
/* Para calibrar Kp, Ki e Kd: calibra-se sempre de cima para baixo. Primeiro calibra-se
o Kp com o Ki = 0 e o Kd = 0. Testem várias vezes com valores diferentes.
Quando ele estiver bom, alterem o Ki e assim sucessivamente*/
/*Melhor conjunto de constantes encontrado:
*/
//Proporionalidade
double Kp = 14; //Contante de proporcionalidade
//Derivada
double errorTable[] = {-ERROR_LEVEL_2, -ERROR_LEVEL_1, 0, ERROR_LEVEL_1, ERROR_LEVEL_2};
double Kd = 100;
int D;
double prevError;
//Constantes
double ROBOT_DIAMETER = 0.120; //Em m
double ROBOT_RADIUS;
double MAX_SPEED =  0.15; //Em m/s
// Velocidades
int speed = 65; //ercentagem da velocidade máxima do motor. Velocidade em linha reta
int turningSpeed = 40; //Velocidade a virar.
//Histórico de medidas
volatile int sensor;
//Contadores. São limitados inferiormente a 0, i.e., counter >= 0
int rightCounter = 0;
int leftCounter = 0;
int deadEndCounter = 0;

/********************************* LÓGICA *********************************/
int STACK_MAXSIZE = 300;
//STACK DO IDENTIFICADORES
int idStack[300];     
int idStackTop = -1;
//STACK DO NUMERO DE RAMOS POR INVESTIGAR
int branchStack[8];     
int branchStackTop = -1;

//**************************** Funções auxiliares (sensores,etc.) ****************************
/* 0 <= i <= 4*/
int sensorGet(int i){ return (sensor >> (4-i)) & 0x01; }
int turnDetected(){ return rightDetected() || leftDetected(); }
void readSensors(){ sensor = readLineSensors(0); }
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
	while(!sensorGet(4)) {readSensors();}	
	while(!detectedLineAhead()){ readSensors(); }
	setVel2(0,0);
	delay(250);
	led(2,0);
}
void turnLeft(){
	led(3,1);
	setVel2(-turningSpeed,turningSpeed);
	while(!sensorGet(0)) {readSensors();}
	while(!detectedLineAhead()){ readSensors(); }
	setVel2(0,0);
	delay(250);
	led(3,0);
}
void invertDirection(){
	led(4,1);
	millis = 0;
	setVel2(-turningSpeed,turningSpeed); //Vira à esquerda até encontrar a linha de novo
	while(!sensorGet(0)) {readSensors();}
	while(!detectedLineAhead() || millis < 850){ readSensors(); }
	setVel2(0,0);
	delay(1000);
	led(4,0);
}

//**************************** Algoritmos para percorrer o labirinto ****************************
/* Segue as indicações da stack. Assume que esta já está preenchida */
void chooseBestPath(){
	int stackIndex = 0;
	while(!stopButton()){
		readSensors();
		adjust();
		if(turnDetected()){
			int nextMove = idStack[stackIndex++];
			switch(nextMove){
				case R: //vira à direita
					turnRight();
					break;
				case L: //vira à esquerda
					turnLeft();
					break;
				case S: //vai em frente
					break;
			}
		}
	}
}

/* Algoritmo para preencher a idStack. Vai virar sempre à direita */
void findBestPath(){

	const static int countAim = 6;
	static int countR, countL, countC = 0;
	static int turnDetected = 0;
	static int isInverted = 0;

	while(!stopButton()) {
		printf("idPeek = %d | branchPeek = %d\n",idStackPeek(),branchStackPeek());
		readSensors();
		adjust();
		//printInt(sensor, 2 | 5 << 16);
		//printf("\n");

		if(!turnDetected && ((sensor & 0x11) != 0)) turnDetected = 1;

		if(turnDetected) {

			if((sensor & 0x11) == 0) {

				fillTheStack(countC,countR,countL,countAim,isInverted);
				countL = 0; countR = 0; countC = 0;
				turnDetected = 0;
			} else {
				if((sensor & 0x10) != 0) countL++;
				if((sensor & 0x01) != 0) countR++;
				if((sensor & 0x0E) != 0) countC++;
			}

		} else if(sensor == 0) {
			invertDirection();
			isInverted = 1;
		}

	}
}

void fillTheStack(int countC, int countR, int countL, int countAim, int isInverted){
	// FUNCAO PARA DETETAR O QUE E A FAZER
	if((countC >= GOAL_CONSTANT) && ((countR >= GOAL_CONSTANT) || (countL >= GOAL_CONSTANT))) { //Encontrou a meta
		waitingStart(); //TERMINOU A INVESTIGAÇÃO DO LABIRINTO E DESCOBRIU A META
		//A stack deve conter o caminho mais curto para a meta
	} else if((countR >= countAim) && (countL >= countAim)) { //Biforcação ou cruzamento
		if(detectedLineAhead()){ // É cruzamento
			if(!isInverted){ //notifica as stacks de que chegou a um cruzamento novo
				idStackPush(R);
				branchStackPush(3);
			}else{ //decrementa o numero de ramos visitados
				isInverted = 0;
				int numLeftToVisit = branchStackPop();
				numLeftToVisit -= 1;
				branchStackPush(numLeftToVisit);
				if(numLeftToVisit == 2){
					idStackPush(S);
				}else if(numLeftToVisit == 1){
					idStackPush(L);
				}else if(numLeftToVisit == 0){
					idStackPop();
					branchStackPop();
					isInverted = 1; //não encontrou a meta até aqui. vai embora desse
				}
			}
		}else{ //É biforcação em T
			if(!isInverted){
				idStackPush(R);
				branchStackPush(2);
			}else{
				int numLeftToVisit = branchStackPop();
				numLeftToVisit -= 1;
				branchStackPush(numLeftToVisit);
				if(numLeftToVisit != 0){
					isInverted = 0;
					idStackPop(); //faz pop do R
					idStackPush(S); //push do S
				}else{
					idStackPop();
					branchStackPop();
				}
			}
		}
		turnRight();
	} else if((countR >= countAim) && (countL < countAim)) { //Curva à direita
		if(detectedLineAhead()){ //biforcação com curva à direita |-
			if(!isInverted){
				idStackPush(R);
				branchStackPush(2);
			}else{
				idStackPop(); //Faz pop do S
				idStackPush(L);
				branchStackPop();
				branchStackPush(1);

			}
		}else{ //Curva simples à direita
			if(!isInverted){ //se estiver a descobrir caminho
				idStackPush(R);
				branchStackPush(1);
			}else{ //se estiver a regressar de um dead end
				idStackPop();
				branchStackPop();
			}
		}
		turnRight();
	} else if((countL >= countAim) && (countR < countAim)) {
		if(detectedLineAhead()){ //biforcação com curva à esquerda -|
			if(!isInverted){
				idStackPush(S);
				branchStackPush(2);
			}else{
				idStackPop();
				idStackPush(L);
				branchStackPop();
				branchStackPush(1);
			}
		}else{ //Curva simples à esquerda
			if(!isInverted){ //se estiver a descobrir caminho
				idStackPush(L);
				branchStackPush(1);
			}else{ //se estiver a regressar de um dead end
				idStackPop(); //Faz pop do R
				branchStackPop();
			}
			turnLeft();
		}
	}
}

//**************************** Funções para controlar o estado do robot ****************************
//Estas funções tem o objetivo de: inicializar o robot (timer,variaveis,etc.),
//decidir se o robot está à espera do botão start e determinar se o robot já conhece o caminho mais rápido
//ou se ainda vai percorrer o labirinto pela primeira vez

void run();
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
	if(idStackTopIsEmpty())
		findBestPath(); //Preenche a idStack
	else
		chooseBestPath();
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

/********************************** STACK RELATED METHODS ********************************/          
//////////////////// ID STACK /////////////////////
int idStackTopIsEmpty() {

   if(idStackTop == -1)
      return 1;
   else
      return 0;
}
   
int idStackIsFull() {

   if(idStackTop == STACK_MAXSIZE)
      return 1;
   else
      return 0;
}

int idStackPeek() {
   return idStack[idStackTop];
}

int idStackPop() {
   int data;
	
   if(!idStackTopIsEmpty()) {
      data = idStack[idStackTop];
      idStackTop = idStackTop - 1;
      return data;
   } else {
      printf("Could not retrieve data, Stack is empty.\n");
   }
   return -1;
}

void idStackPush(int data) {

   if(!idStackIsFull()) {
      idStackTop = idStackTop + 1;   
      idStack[idStackTop] = data;
   } else {
      printf("Could not insert data, Stack is full.\n");
   }
}

void idStackPrint(){
	int i;
	printf("----begin----\n");
	for(i = 0; i <= idStackTop; i++){
		printf("%d\n",idStack[i]);
	}
	printf("----end----\n");
}

//////////////////// BRANCH STACK /////////////////////

int branchStackIsEmpty() {

   if(branchStackTop == -1)
      return 1;
   else
      return 0;
}
   
int branchStackIsFull() {

   if(branchStackTop == STACK_MAXSIZE)
      return 1;
   else
      return 0;
}

int branchStackPeek() {
   return branchStack[branchStackTop];
}

int branchStackPop() {
   int data;
	
   if(!branchStackIsEmpty()) {
      data = branchStack[branchStackTop];
      branchStackTop = branchStackTop - 1;   
      return data;
   } else {
      printf("Could not retrieve data, Stack is empty.\n");
   }
   return -1;
}

void branchStackPush(int data) {

   if(!branchStackIsFull()) {
      branchStackTop = branchStackTop + 1;   
      branchStack[branchStackTop] = data;
   } else {
      printf("Could not insert data, Stack is full.\n");
   }
}

void branchStackPrint(){
	int i;
	printf("----begin----\n");
	for(i = 0; i <= branchStackTop; i++){
		printf("%d\n",branchStack[i]);
	}
	printf("----end----\n");
}
