/**
 * \file
 * \brief example_pathfinder.c (PathFinder - basic example)
 */

// ****************************************************************************
// EXAMPLE_PATHFINDER.C
//
// To compile this example: pcompile example_pathfinder.c mr32.c
// ****************************************************************************
//
#include "mr32.h"

int main(void)
{
   int groundSensor;
   int vel_max, vel_curva_st, vel_curva_hd, vel_rotl, vel_roth;

   vel_max = 65;
   vel_curva_st = 40;
   vel_curva_hd = 10;
   vel_rotl = 20;
   vel_roth = 50;


   initPIC32();
   closedLoopControl( false );
   setVel2(0, 0);

   printf("PathFinder example\n\n\n");

   while(1)
   {
      printf("Press start to continue\n");
      while(!startButton())
      {
         waitTick80ms();
         groundSensor = readLineSensors(0);	// Read ground sensor
         printInt(groundSensor, 2 | 5 << 16);
         printf("\n");
      }

      do
      {
         groundSensor = readLineSensors(0);	// Read ground sensor

         switch(groundSensor)
         {
            case 0x04:  // 0b00100:
               setVel2(vel_max, vel_max);
               break;
            case 0x0C:  // 0b01100:
               setVel2(vel_curva_st, vel_max);
               break;
            case 0x08:  // 0b01000:
               setVel2(vel_curva_hd, vel_max);
               break;
            case 0x10:  // 0b10000:
               setVel2(-vel_rotl, vel_roth);
               break;
            case 0x06:  // 0b00110:
               setVel2(vel_max, vel_curva_st);
               break;
            case 0x02:  // 0b00010:
               setVel2(vel_max, vel_curva_hd);
               break;
            case 0x01:  // 0b00001:
               setVel2(vel_roth, -vel_rotl);
               break;
            default:
               break;
         }
      } while(!stopButton());
      setVel2(0, 0);
   }
   return 0;
}

