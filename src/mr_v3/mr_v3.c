#include "mr32.h"

volatile int millis = 0;

// Velocidades
int speed = 40; //Velocidade em linha reta
int adjust = 8; //Velocidade de ajuste. Vai ser somado a uma roda e subtraido à outra
int turningSpeed = 40; //Velocidade a virar
int sensor;

//Informação sobre o estado do jogo
//Número de tentativas. Incrementado no final de cada tentativa
static int numTries = 0; //Não deve ser feito o reset desta variável

//**************************** Funções auxiliares (sensores,etc.) ****************************
void stopRobot(){ setVel2(0,0); }
void adjustToTheLeft(){ setVel2(speed-adjust,speed+adjust); };
void adjustToTheRight(){ setVel2(speed+adjust,speed-adjust); }
void goFoward(){ setVel2(speed,speed); }
void resetAllVariables(){ stopRobot(); }
int centerSensors(){ return sensor ==  0b00110 || sensor ==  0b01100 || sensor ==  0b00100; }
//**************************** Algoritmos para percorrer o labirinto ****************************«
/* Algoritmo para preencher a stack. Vai virar sempre à direita */
void findBestPath(){
	int finished = 0;
	while(!finished && !stopButton()) {
		sensor = readLineSensors(0);
		leds(sensor);
		//if(centerSensors()){ //If it is going forward
		switch(sensor){
				case 0b00100:
				case 0b01110:
					goFoward();
					break;
				case 0b01100:
				case 0b01000:
					adjustToTheLeft();
					break;
				case 0b00110:
				case 0b00010:
					adjustToTheRight();
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
	led(2,1);
	while(!startButton()); //Enquanto o botão start nao for premido
	led(2,0);
	run(); //Executa uma nova tentativa
}

/*Configurações do robot*/
void configureRobot(){
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

/*Atualiza o temporizador*/
void _int_(4) isr_T1() {
	millis++;
	IFS0bits.T1IF = 0;

}



