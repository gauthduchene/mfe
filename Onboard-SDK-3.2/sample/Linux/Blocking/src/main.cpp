/*! @file main.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  New Linux App for DJI Onboard SDK. 
 *  Provides a number of convenient abstractions/wrappers around core API calls.
 *
 *  Calls are blocking; the calling thread will sleep until the
 *  call returns. Use Core API calls or another sample if you 
 *  absolutely need non-blocking calls. 
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

//System Headers
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fstream>
#include <stdio.h> 
#include <math.h>


//DJI Linux Application Headers
#include "LinuxSerialDevice.h"
#include "LinuxThread.h"
#include "LinuxSetup.h"
#include "LinuxCleanup.h"
#include "ReadUserConfig.h"
#include "LinuxMobile.h"
#include "LinuxFlight.h"
#include "LinuxInteractive.h"
#include "LinuxWaypoint.h"
#include "LinuxCamera.h"

//DJI OSDK Library Headers
#include <DJI_Follow.h>
#include <DJI_Flight.h>
#include <DJI_Version.h>
#include <DJI_WayPoint.h>

//Local Mission Planning Suite Headers
//#include <MissionplanHeaders.h>


#define RAD2DEG 57.2957795131
using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;
CoreAPI* api;
Flight* flight;
int c;
float radioValue =6;
float *pointerRadio;
ofstream fileRadio;
int *pointerNumber;
int number=0;

//! Main function for the Linux sample. Lightweight. Users can call their own API calls inside the Programmatic Mode else on Line 68. 
int main(int argc, char *argv[])
{
	
	pointerRadio = &radioValue;
	pointerNumber = &number;
  //! Instantiate a serialDevice, an API object, flight and waypoint objects and a read thread.
  LinuxSerialDevice* serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
  api = new CoreAPI(serialDevice);
  flight = new Flight(api);
  WayPoint* waypointObj = new WayPoint(api);
  Camera* camera = new Camera(api);
  LinuxThread read(api,flight,pointerRadio,pointerNumber,argc,argv, 2);
  LinuxThread data(api,flight,pointerRadio,pointerNumber,argc,argv, 4); // create thread for save data
  LinuxThread key(api,flight,pointerRadio,pointerNumber,argc,argv, 5);
  LinuxThread radio(api,flight,pointerRadio,pointerNumber,argc,argv,6);  // create thread to control radio
 fileRadio.open("valueFromRadio.csv");
  //! Setup
  int setupStatus = setup(serialDevice, api, &read,&data,&key, &radio);
  if (setupStatus == -1)
  {
    std::cout << "This program will exit now. \n";
    return 0;
  }
  //! Set broadcast Freq Defaults
  unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
  usleep(500000);
  //! Mobile Mode
  if (argv[1] && !strcmp(argv[1],"-mobile"))
  {
    std::cout << "Listening to Mobile Commands\n";
    mobileCommandSpin(api, flight, waypointObj, camera, argv, argc);
  }
  //! Interactive Mode
  else if (argv[1] && !strcmp(argv[1], "-interactive"))
  {
    if (argc > 3)
      interactiveSpin(api, flight, waypointObj, camera, std::string(argv[2]), std::string(argv[3]));
    else if (argc == 3)
      interactiveSpin(api, flight, waypointObj, camera, std::string(argv[2]), std::string(""));
    else
      interactiveSpin(api, flight, waypointObj, camera, std::string(""), std::string(""));
  }
  //! Programmatic Mode - execute everything here without any interactivity. Useful for automation.
  else if (argv[1] && !strcmp(argv[1], "-programmatic"))
  {
    /*! Set a blocking timeout - this is the timeout for blocking API calls
        to wait for acknowledgements from the aircraft. Do not set to 0.
    !*/
    int blockingTimeout = 1; //Seconds
    
    //! Monitored Takeoff
    ackReturnData takeoffStatus = monitoredTakeoff(api, flight, blockingTimeout);

    //! Set broadcast Freq Defaults
    unsigned short broadcastAck = api->setBroadcastFreqDefaults(1);
    
    //! If the aircraft took off, continue to do flight control tasks 
    if (takeoffStatus.status == 1)
    {

      /*! This is where you can add your own flight functionality.
          Check out LinuxWaypoint and LinuxFlight for available APIs. 
          You can always execute direct Onboard SDK Library API calls
          through the api object available in this example.
      !*/ 
		PositionData curPosition;
		PositionData originPosition;
 		DJI::Vector3dData curLocalOffset; 
		DJI::EulerAngle curEuler;
	  std::cout << "\n avant le move\n";
    float speedX=0.5;
    float speedY=0.5;
    float speedZ=0.5;
   	curPosition = api->getBroadcastData().pos;
		originPosition = curPosition;
    int currentValueX;
    int currentValueY;
    int currentValueZ;
    int currentValueYAW;
    int done=0;
    int tour=0;
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
    char inputchar;
    ackReturnData takeControlStatus;
    ackReturnData releaseControlStatus;
    ackReturnData takeoff;
    ackReturnData land ;
    Angle yaw;
    Angle oldYaw;
    Angle old;
    yaw=flight->getYaw();
    int quarter=0;
    fileRadio << "Number" <<","<<"yaw"  <<","<< "radioValue" <<","<< "Position x"  <<","<< "Position y"  <<","<< "Position z"<<"\n";
    while (tour==0){
    
    std::cout << " please enter a letter. a for take control, n for a tour, l for a line, t for takeoff and x for landing " << std::endl;
     cin >> inputchar;	

switch (inputchar)
    {
	  case 'a':
        takeControlStatus = takeControl(api);
        break;	
      case 'n':
      number++;
      yaw=flight->getYaw();
      yaw=RAD2DEG*yaw+180;
      oldYaw=yaw;
      old=yaw;
        while( quarter<5 ) {
		yaw=flight->getYaw();
		yaw=RAD2DEG*yaw+180;	
        int status1=moveWithVelocity(api,flight,0,0,0, 10 ,1500, 3, 0.1);
        if ( oldYaw <90 && yaw >= 90){
			cout<< " First quarter complete with an angle of : "<< yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw <180 && yaw >=180){
			cout<< " Second quarter complete  with an angle of : "<< yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw <270 && yaw >= 270){
			cout<< " Third quarter complete  with an angle of : " << yaw << endl;
			quarter=quarter+1;
		}
		else if ( oldYaw>350  && yaw <=10){
			cout<< " Fourth quarter complete  with an angle of : " << yaw<< endl;
			quarter=quarter+1;
		}
		if(fmod(yaw,5)<1 && abs(yaw-old)>4 ){
			curPosition =flight -> getPosition();
            localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
			fileRadio <<number << "," << yaw  <<","<< radioValue <<","<< curLocalOffset.x  <<","<< curLocalOffset.y  <<","<< curLocalOffset.z<<"\n";
			old=yaw;
		}
        
        oldYaw=yaw;
		
	} quarter=0;
	cout << " apres le while     " << yaw<< endl;
        break;
      case 'l':
        while(curLocalOffset.x<3 ){
		int status1=moveWithVelocity(api,flight,speedX,0,0,0 ,1500, 3, 0.1);
		curPosition = api->getBroadcastData().pos;
  	curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
   }
        break;
      case 't':
         takeoff = monitoredTakeoff(api, flight, blockingTimeout);
        break;
      case 'x':
         land = landing(api, flight,blockingTimeout);
        tour=1;
        break;
      case 'r':
       releaseControlStatus = releaseControl(api);
       break;
      //! Do aircraft control - Waypoint example. 
     // wayPointMissionExample(api, waypointObj,blockingTimeout);
	}
	
}
      //! Land
      ackReturnData landingStatus = landing(api, flight,blockingTimeout);
    }
    else 
    {
      //Try to land directly
      ackReturnData landingStatus = landing(api, flight,blockingTimeout);
    }
  }
  //! No mode specified or invalid mode specified" 
  else
    std::cout << "\n Usage: ./djiosdk-linux-sample [MODE] [TRAJECTORY] [GAIN TUNING]\n"
                 "\n"
                 "[MODE] : \n"
                 "-mobile      : Run in Mobile Data Transparent Transmission mode\n"
                 "-interactive : Run in a Terminal-based UI mode\n"
                 "-programmatic: Run without user input, use if you have put automated\n"
                 "               calls in the designated space in the main function. \n"
                 "               By default this mode will execute an automated waypoint\n"
                 "               mission example, so be careful.\n\n"
                 "[TRAJECTORY] : \n"
                 "path_to_json_file : Optionally, supply a json file with parameters for executing a\n"
                 "                    trajectory planned with the DJI Trajectory SketchUp Plugin\n\n";
                 "[GAIN TUNING] : \n"
                 "path_to_json_file : Optionally, supply a json file with custom controller gains for\n"
                 "                    executing precision missions on loaded aircraft\n\n";
  //! Cleanup
  int cleanupStatus = cleanup(serialDevice, api, flight, &read);
  if (cleanupStatus == -1)
  {
    std::cout << "Unable to cleanly destroy OSDK infrastructure. There may be residual objects in the system memory.\n";
    return 0;
  }
  std::cout << "Program exited successfully." << std::endl;
  //myfile.close();
  return 0;
}


/*



void *Linux::save_data (void *param) {
int i=0;
int blockingTimeout = 1; 
  unsigned short dataFlag;
  TimeStampData timeStamp;
  QuaternionData q;
  CommonData ya;
  CommonData acc;
  VelocityData v;
  PositionData pos;
  MagnetData mag;
  GPSData gps;
  RTKData rtk;
  RadioData rc;
  FlightStatus status; //! @todo define enum
  BatteryData battery;
  CtrlInfoData ctrlInfo;
  Angle yaw;
  Angle roll;
  Angle pitch;
  PositionData curPosition;
  PositionData originPosition;
 	DJI::Vector3dData curLocalOffset; 
  DJI::EulerAngle curEuler;
usleep(500000);
  curPosition = flight -> getPosition();

	originPosition = curPosition;
 localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
	myfile.open ("test.csv");
	myfile << "Quaternion q0,Quaternion q1,quaternion q2,quaternion q3,velocity x,velocity y, velocity z,latitude,longitude,altitude,height, acceleration x,acceleration y, acceleration z, mag x, mag y, mag z,Yaw,Roll,Pitch,\n";
pos=flight -> getPosition();
    while (i<100){
        q=flight   -> getQuaternion();
        v=flight   -> getVelocity();
        pos=flight -> getPosition();
	acc=flight -> getAcceleration();
	ya=flight  -> getYawRate();
	mag=flight -> getMagnet();
        yaw=flight ->getYaw();
	roll=flight ->getRoll();
        pitch=flight ->getPitch();
        curPosition =flight -> getPosition();
curEuler = Flight::toEulerAngle(q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);

	//gps=flight -> 
	// rtk
	//rc	

	myfile <<q.q0  <<","<< q.q1 <<","<< q.q2 <<","<< q.q3 <<","<< v.x <<","<< v.y <<","<< v.z <<","<< pos.latitude <<","<< pos.longitude <<","<< pos.altitude <<","<< pos.height <<","<< acc.x <<","<< acc.y <<","<< acc.z <<","<< mag.x <<","<< mag.y <<","<< mag.z <<","<<yaw <<","<<roll<<","<<pitch<< ","<< curLocalOffset.x << ","<< curLocalOffset.y << ","<<curLocalOffset.z <<",\n";

	i=i+1;
	usleep(500000);
	
}	

}

void *LinuxThread::key_call (void *param) {
while(1){
c=getchar();

}
}

*/

