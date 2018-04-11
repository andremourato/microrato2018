/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2015 Universidade de Aveiro

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

import java.io.*;
import java.net.*;
import java.util.*;
import java.util.Vector;

import ciberIF.*;


/**
 * example of a basic agent
 * implemented using the java interface library.
 */
public class jClient {

    ciberIF cif;

    enum State {RUN, WAIT, RETURN}

    public static void main(String[] args) {

        String host, robName;
        int pos; 
        int arg;

        //default values
        host = "localhost";
        robName = "jClient";
        pos = 1;


        // parse command-line arguments
        try {
            arg = 0;
            while (arg<args.length) {
                if(args[arg].equals("-pos")) {
                        if(args.length > arg+1) {
                                pos = Integer.valueOf(args[arg+1]).intValue();
                                arg += 2;
                        }
                }
                else if(args[arg].equals("-robname")) {
                        if(args.length > arg+1) {
                                robName = args[arg+1];
                                arg += 2;
                        }
                }
                else if(args[arg].equals("-host")) {
                        if(args.length > arg+1) {
                                host = args[arg+1];
                                arg += 2;
                        }
                }
                else throw new Exception();
            }
        }
        catch (Exception e) {
                print_usage();
                return;
        }
        
        // create client
        jClient client = new jClient();

        client.robName = robName;

        // register robot in simulator
        client.cif.InitRobot(robName, pos, host);
        
        // main loop
        client.mainLoop();
        
    }

    // Constructor
    jClient() {
            cif = new ciberIF();
            beacon = new beaconMeasure();

            beaconToFollow = 0;
            ground=-1;

            state = State.RUN;
    }

    /** 
     * reads a new message, decides what to do and sends action to simulator
     */
    public void mainLoop () {

        while(true) {
                cif.ReadSensors();
                decide();
        }
    }

    public void wander(boolean followBeacon) {
        if(irSensor0>4.0 || irSensor1>4.0 ||  irSensor2>4.0) 
                cif.DriveMotors(-0.1,+0.1);
        else if(irSensor1>0.7) cif.DriveMotors(0.15,0.0);
        else if(irSensor2>0.7) cif.DriveMotors(0.0,0.15);
        else if(followBeacon && beacon.beaconVisible && beacon.beaconDir > 20.0) 
            cif.DriveMotors(0.0,0.1);
        else if(followBeacon && beacon.beaconVisible && beacon.beaconDir < -20.0) 
            cif.DriveMotors(0.1,0.0);
        else cif.DriveMotors(0.15,0.15);
    }

    /**
     * basic reactive decision algorithm, decides action based on current sensor values
     */
    public void decide() {
            if(cif.IsObstacleReady(0))
                    irSensor0 = cif.GetObstacleSensor(0);
            if(cif.IsObstacleReady(1))
                    irSensor1 = cif.GetObstacleSensor(1);
            if(cif.IsObstacleReady(2))
                    irSensor2 = cif.GetObstacleSensor(2);

            if(cif.IsCompassReady())
                    compass = cif.GetCompassSensor();
            if(cif.IsGroundReady())
                    ground = cif.GetGroundSensor();

            if(cif.IsBeaconReady(beaconToFollow))
                    beacon = cif.GetBeaconSensor(beaconToFollow);

            //System.out.println("Measures: ir0=" + irSensor0 + " ir1=" + irSensor1 + " ir2=" + irSensor2 + "\n");

            //System.out.println(robName + " state " + state);

            switch(state) {
                 case RUN:    /* Go */
                     if( cif.GetVisitingLed() ) state = State.WAIT;
                     if( ground == 0 ) {         /* Visit Target */
                         cif.SetVisitingLed(true);
                         System.out.println(robName + " visited target at " + cif.GetTime() + "\n");
                     }

                     else {
                         wander(false);
                     }
                     break;
                 case WAIT: /* set returning led and check that it is on */
                     cif.SetReturningLed(true);
                     if(cif.GetVisitingLed()) cif.SetVisitingLed(false);
                     if(cif.GetReturningLed()) state = State.RETURN;

                     cif.DriveMotors(0.0,0.0);
                     break;
                 case RETURN: /* Return to home area */
                     cif.SetVisitingLed(false);
                     cif.SetReturningLed(false);
                     wander(false);
                     break;

            }


            //for(int i=1; i<6; i++)
            //  if(cif.NewMessageFrom(i))
            //      System.out.println("Message: From " + i + " to " + robName + " : \"" + cif.GetMessageFrom(i)+ "\"");


            //cif.Say(robName);

            if(cif.GetTime() % 2 == 0) {
                 cif.RequestIRSensor(0);
                 cif.RequestIRSensor(1);
                 cif.RequestIRSensor(2);
                 cif.RequestGroundSensor();
            }
            else {
               String [] sensList = {"IRSensor1","IRSensor2","Compass","GPS"};
               cif.RequestSensors(sensList);
            }
    }

    static void print_usage() {
             System.out.println("Usage: java jClient [-robname <robname>] [-pos <pos>] [-host <hostname>[:<port>]]");
    }

    private String robName;
    private double irSensor0, irSensor1, irSensor2, compass;
    private beaconMeasure beacon;
    private int    ground;
    private boolean collision;

    private State state;

    private int beaconToFollow;
};

