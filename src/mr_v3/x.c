#include "mr32.h"

int main(void) {

	closedLoopControl(false); //Ativa a estabilização por PID
	initPIC32();

	while(1) {
		setVel2(70, 70);
	}

}
