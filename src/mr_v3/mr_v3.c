#include "mr32.h"

#define ERROR_LEVEL_1 2 //Melhor: 2
#define ERROR_LEVEL_2 3 //Melhor: 3
#define NUMBER_OF_SAMPLES 10
//**************************** Funções auxiliares Declarações ****************************
double getRealSpeed();
void stopRobot();
void resetAllVariables();
void adjust(int);
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
int turningSpeed = 50; //Velocidade a virar

//Histórico de medidas
volatile int sensor;
int sensorHistory[N];

//Informação sobre o estado do jogo
//Número de tentativas. Incrementado no final de cada tentativa
static int numTries = 0; //Não deve ser feito o reset desta variável

//**************************** Funções auxiliares (sensores,etc.) ****************************
double getRealSpeed(){ return speed*MAX_SPEED/100.0; } //a variavel speed é apenas a percentagem da velocidade máxima
int detectedLineAhead(){return sensor == 0b00100 || sensor == 0b01100 || sensor == 0b01000 ||
							   sensor == 0b00110 || sensor == 0b00010;}
void stopRobot(){ setVel2(0,0); }
void resetAllVariables(){ stopRobot(); }
//Calcula o PID
void adjust(int sensorValue){
	int middleSensors = (sensorValue & 0x0E) >> 1; //Gets the values of the 3 middle sensors
	//printInt(middleSensors,2 | (3 << 16));
	//printf("\n");
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

//**************************** Algoritmos para percorrer o labirinto ****************************
/* Algoritmo para preencher a stack. Vai virar sempre à direita */
void findBestPath(){
	int finished = 0; //variavel que dará por terminado o jogo. Fica a 1 quando encontrou o objetivo
	while(!finished && !stopButton()) {
		sensor = readLineSensors(0);
		adjust(sensor);
		/*switch(sensor){
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
				case 0b11000:
				case 0b01000: //Está a fugir para a direita
					adjust(-2);
					break;
				case 0b11100:
				case 0b01100:
					adjust(-1);
					break;
				case 0b10100:
				case 0b11110:
				case 0b00100:
					adjust(0);
					break;
				case 0b10110:
				case 0b00110:
					adjust(1);
					break;
				case 0b10010:
				case 0b00010: //Está a fugir para a esquerda
					adjust(2);
					break;
				case 0b01110: //Pode ter que compensar para a esquerda ou direita
					adjust(3);
		}*/
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
	calculateVariables();
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

void calculateVariables(){

}

/*Atualiza o temporizador*/
void _int_(4) isr_T1() {
	millis++;
	IFS0bits.T1IF = 0;

}



