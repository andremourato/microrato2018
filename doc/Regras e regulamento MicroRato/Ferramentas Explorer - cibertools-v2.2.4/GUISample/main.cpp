/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2016 Universidade de Aveiro

    ciberRatoToolsSrc is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ciberRatoToolsSrc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/* main.cpp
 *
 * Basic Robot Agent
 * simple version for demonstration
 *
 * For more information about the CiberRato Robot Simulator 
 * please see http://microrato.ua.pt/ or contact us.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <iostream>

using std::cerr;

#include <qapplication.h>
#include "../libRobSock/RobSock.h"
#include "sampapp.h"
#include "robview.h"


/* Calculate the power of left and right motors */
void DetermineAction(int beaconToFollow, double *lPow, double *rPow)
{
    static int counter=0;

    static double left, right, center;
    bool   beaconReady;
    static struct beaconMeasure beacon;
    static int    Ground; 
    static bool   Collision ;
    
    /*Access to values from Sensors - Only ReadSensors() gets new values */
    if(IsObstacleReady(LEFT))
        left      = GetObstacleSensor(LEFT);
    if(IsObstacleReady(RIGHT))
        right     = GetObstacleSensor(RIGHT);
    if(IsObstacleReady(CENTER))
        center    = GetObstacleSensor(CENTER);

    beaconReady = IsBeaconReady(beaconToFollow);
    if(beaconReady) {
         beacon    = GetBeaconSensor(beaconToFollow);
    }

    if(IsGroundReady())
       Ground    = GetGroundSensor();
    if(IsBumperReady())
       Collision = GetBumperSensor();

    if(center>4.5 || right>4.5 || left>4.5 || Collision) { /* Close Obstacle - Rotate */
        if(counter % 400 < 200) {
           *lPow=0.06;
           *rPow=-0.06; }
        else {
           *lPow=-0.06;
           *rPow=0.06; }
    }
    else if(right>1.5) { /* Obstacle Near - Avoid */
        *lPow=0.0;
        *rPow=0.05;
    }
    else if(left>1.5) {
        *lPow=0.05;
        *rPow=0.0;
    }
    else { 
        if(beaconReady && beacon.beaconVisible) {
            if(beacon.beaconDir>20.0) { /* turn to Beacon */
               *lPow=0.0;
               *rPow=0.1;
            }
            else if(beacon.beaconDir<-20.0) {
               *lPow=0.1;
               *rPow=0.0;
            }
            else { /* Full Speed Ahead */
               *lPow=0.1;
               *rPow=0.1;
            }
        }
        else { /* Full Speed Ahead */
           *lPow=0.1;
           *rPow=0.1;
        }
    }

    counter++;
}

/** SampApp methods **/

SampApp::SampApp(int &argc, char*argv[], char *robot_name) : QApplication(argc,argv)
{
    strncpy(rob_name, robot_name, 19);
    rob_name[19]='\0';

    beaconToFollow = 0; // start by finding target at beacon 0
}

#define RUN        1
#define STOP       2
#define WAIT       3
#define RETURN     4
#define FINISHED   5

void SampApp::act(void)
{
    static int state=STOP,stoppedState=RUN;
    double lPow, rPow;

    ReadSensors();

    if(GetFinished()) /* Simulator has received Finish() or Robot Removed */
    {
        printf("%s Exiting %d\n",rob_name, GetTime());
        exit(0);
        return;
    }
    if(state==STOP && GetStartButton()) state=stoppedState;  /* Start     */
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


    // Communication
    //Say(rob_name);

}


int main( int argc, char** argv )
{
    char host[100]="localhost";
    char rob_name[20]="GUISample";
    int rob_id=1;

    printf(" GUISample Robot \n Copyright 2002-2016 Universidade de Aveiro\n");
    fflush(stdout);

    /* processing arguments */
    while (argc > 2) // every option has a value, thus argc must be 1, 3, 5, ...
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
               argc=0; // error message will be printed
        }
        else
        {
                break; // the while
        }
        argc -= 2;
        argv += 2;
    }

    if (argc != 1)
    {
        fprintf(stderr, "Bad number of parameters\n"
                "SYNOPSIS: GUISample [-host hostname] [-robname robotname] [-pos posnumber]\n");
        return 1;
    }

    // Create Qt application - Must be before InitRobot
    SampApp app( argc, argv, rob_name );
    qApp->addLibraryPath("../libRobSock");

    /* Connect Robot to simulator */
    double irSensorAngles[4] = { 0.0, 60.0, -60.0, 180.0};
    if(InitRobot2(rob_name,rob_id,irSensorAngles,host)==-1) {
          printf("%s Failed to connect\n",rob_name);
          exit(1);
    }
    printf("%s Connected\n",rob_name);
    fflush(stdout);

    // Connect event NewMessage to handler act()
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), &app, SLOT(act()));
    
    // create robot display widget
    RobView robView(irSensorAngles, rob_name);

    // Connect event NewMessage to handler redrawRobot()
    QObject::connect((QObject *)(Link()), SIGNAL(NewMessage()), &robView, SLOT(redrawRobot()));
    
    robView.show();

    // process events
    return app.exec();
}
