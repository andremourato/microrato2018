#include "include/stdlib.h"
#include "mr32.h"

int getOppositeWay(int);

int main(void)
{

	TRISE = TRISE & 0xFFF0;
	int sensor;

	initPIC32();
	closedLoopControl( false );
	setVel2(0, 0);	

	int block = 0;

	int turnCounter = 0;
	int turning = 0;
	int time = 0;

	int size = 0;
	int turns[50] = {}; // 1 representa curvas para a esquerda, 2 para a direita, e 3 para situacoes em que ha circuito para os dois lados

	int inversed = 0;

	int fourWays = 0;

	while(1) {
		
		while(!startButton()) {
		

			if(size != 0) LATE = (LATE & 0xFFF0) | turns[size - 1];	
			delay(5000);
			LATE = (LATE & 0xFFF0) | size;
			delay(5000);

		}
		
		do {

			sensor = readLineSensors(0);
			printInt(sensor, 2 | 5 << 16);
			printStr("\n");
			
			if(block++ > 250) block = 0;
			if(block) { 
				turns[size - 1] = 1;
				inversed = 0;
			}
 			if(turning) turnCounter++;

			if((sensor == 0x1E) || (sensor == 0x18)) { // 11000 e 11110 para 11100
				sensor = 0x1C;
			} else if((sensor == 0x0F) || (sensor == 0x03)) { // 00011 e 01111 para 00111
				sensor = 0x07;
			}

			LATE = (LATE & 0xFFF0) | inversed;

			if(inversed) {

				if((sensor == 00111) || (sensor == 11100)) {
					if(turns[size - 1] == 3) {
						block++;
					} else {
						inversed = 1;
						size--;
					}
				}

				if(sensor == 11111) {

					if(turns[size - 1] == 1) { 
						sensor = 11100;	
					} else if(turns[size - 1] == 2) {
						sensor = 00111;	
					} else {
						fourWays++;
					}
					size--;	

					if(fourWays >= 3) {	
						fourWays = 0;
						inversed = 1;
					}
				}

			

			}

			switch(sensor) {

				// EM FRENTE
				case 0x04:	// 00100
				case 0x0E:	// 01110	
					if(turnCounter < 30 && turnCounter > 0) {
						size--;
					}
					turnCounter = 0;
					turning = 0;
					setVel2(30 ,30);
					break;
				
				// AJUSTAR EM FRENTE
				case 0x06:	// 00110
				case 0x02:	// 00010
					if(turnCounter < 30 && turnCounter > 0) {
						size--;
					}
					turnCounter = 0;
					turning = 0;
					setVel2(30, 20);
					break;
				case 0x0C:	// 01100
				case 0x08:	// 01000
					if(turnCounter < 30 && turnCounter > 0) {
						size--;
					}
					turnCounter = 0;
					turning = 0;
					setVel2(20, 30);
					break;	

				// VIRAR
				case 0x1C:	// 11100
					if(block == 0 && !turning) {
						setVel2(0, 50);
						turning = 1;
						turns[size++] = 1;
						if(inversed)
							inversed = 0;
					}
					break;
				case 0x07:	// 00111
					if(block == 0 && !turning) {
						setVel2(50, 0);
						turning = 1;
						turns[size++] = 1;
						if(inversed)
							inversed = 2;
					}
					break;
				case 0x1F:	// 11111
					if(inversed) inversed = 0;
					if(turning && turns[size - 1] != 2 && fourWays < 2) {
						turns[size-1] = 3;
					} else if(!turning) {
						setVel2(50,0);	
						turning = 1;
						if(fourWays < 2)
							turns[size++] = 3;
					}
								
					break;
	
				case 0x00:	// 00000
					if(!turning) {
						if(size == 0) {
							setVel2(0, 0);
						} else {
							setVel2(35, -35);
							inversed = 1;
						}
					}
					break;
				default:
					break;
			}
				

		} while(!stopButton());

		setVel2(0,0);
		inversed = 0;
		turning = 0;

	}	

}

