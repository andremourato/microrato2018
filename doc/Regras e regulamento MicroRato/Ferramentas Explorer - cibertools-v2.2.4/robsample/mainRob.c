
/* mainRob.C
 *
 * Basic Robot Agent
 * Very simple version for demonstration
 *
 * For more information about the CiberRato Robot Simulator 
 * please see http://microrato.ua.pt/ or contact us.
 */

#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "RobSock.h"

#include "robfunc.h"

#define true 1
#define false 0

int main(int argc, char *argv[])
{
    char host[100]="localhost";
    char rob_name[20]="robsample";
    float lPow,rPow;
    int state=STOP, stoppedState=RUN, rob_id = 1;

    printf( " Sample Robot\n Copyright (C) 2001-2016 Universidade de Aveiro\n" );

    /* processing arguments */
    while (argc > 2) /* every option has a value, thus argc must be 1, 3, 5, ... */
    {
        if (strcmp(argv[1], "-host") == 0)
        {
           strncpy(host, argv[2], 99);
           host[99]='\0';
        }
        else if (strcmp(argv[1], "-robname") == 0)
        {
           strncpy(rob_name, argv[2], 19);
           rob_name[19]='\0';
        }
        else if (strcmp(argv[1], "-pos") == 0)
        {
            if(sscanf(argv[2], "%d", &rob_id)!=1)
               argc=0; /* error message will be printed */
        }
        else
        {
                break; /* the while */
        }
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        fprintf(stderr, "Bad number of parameters\n"
                "SYNOPSIS: mainRob [-host hostname] [-robname robotname] [-pos posnumber]\n");

        return 1;
    }

    /* Connect Robot to simulator */
    if(InitRobot(rob_name, rob_id, host)==-1)
    {
       printf( "%s Failed to connect\n", rob_name); 
       exit(1);
    }
    printf( "%s Connected\n", rob_name );
    state=STOP;
    while(1)
    {
      /* Reading next values from Sensors */
      ReadSensors();

      if(GetFinished()) /* Simulator has received Finish() or Robot Removed */
      {
          printf(  "%s Exiting\n", rob_name );
          exit(0);
      }
      if(state==STOP && GetStartButton()) state=stoppedState;  /* Restart     */
      if(state!=STOP && GetStopButton())  {
          stoppedState=state;
          state=STOP; /* Interrupt */
      }

      switch (state) { 
          case RUN:    /* Go */
              if( GetVisitingLed() ) state = WAIT;
              if(GetGroundSensor()==0) {         /* Visit Target */
                  SetVisitingLed(true);
                  printf("%s visited target at %d\n", rob_name, GetTime());
              }

              else {
                  DetermineAction(0,&lPow,&rPow);
                  DriveMotors(lPow,rPow);
              }
              break;
          case WAIT: /* Wait for others to visit target */
              SetReturningLed(true);
              if(GetVisitingLed()) SetVisitingLed(false);
              if(GetReturningLed()) state = RETURN;
              DriveMotors(0.0,0.0);
              break;
          case RETURN: /* Return to home area */
              if(GetVisitingLed()) SetVisitingLed(false);
              SetReturningLed(false);

              // Wander
              DetermineAction(1,&lPow,&rPow);
              DriveMotors(lPow,rPow);

              break;
      }

      //Request Sensors for next cycle
      if(GetTime() % 2 == 0) {
          RequestObstacleSensor(CENTER);
          RequestObstacleSensor(LEFT);
          RequestObstacleSensor(RIGHT);
          RequestGroundSensor();

      }
      else {
          RequestSensors(4, "IRSensor1", "IRSensor2","Compass","GPS");
      }

    }
    return 1;
}

