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

using namespace std;
using namespace DJI;
using namespace DJI::onboardSDK;
CoreAPI* api;
Flight* flight;
int c;
ofstream myfile;

//! Main function for the Linux sample. Lightweight. Users can call their own API calls inside the Programmatic Mode else on Line 68. 
int main(int argc, char *argv[])
{
  //! Instantiate a serialDevice, an API object, flight and waypoint objects and a read thread.
  LinuxSerialDevice* serialDevice = new LinuxSerialDevice(UserConfig::deviceName, UserConfig::baudRate);
  api = new CoreAPI(serialDevice);
  flight = new Flight(api);
  WayPoint* waypointObj = new WayPoint(api);
  Camera* camera = new Camera(api);
  LinuxThread read(api,flight, 2);
  LinuxThread data(api,flight, 4); // create thread for save data
  LinuxThread key(api,flight, 5);
  LinuxThread radio(api,flight,6);  // create thread to control radio

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
localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
std::cout << "valeur de longitude en main : " << curPosition.latitude <<"\n" ;
std::cout<< "la valeur de la position :" << curLocalOffset.x << "\n" ;
  	while(curLocalOffset.x<3 && done==0){
		int status1=moveWithVelocity(api,flight,speedX,0,0,0 ,1500, 3, 0.1);
		curPosition = api->getBroadcastData().pos;
  	curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);

     if('a'==putchar(c)){
 		std::cout<<"parti pour 5 secondes stationnaires";           
		int ackStat=moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0);	   
	  	usleep(5000000);
	   	std::cout<<"fin des 5 secondes";
	   	done=1;
        }
        else if('q'==putchar(c)){
           int ackStat=moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0); //roll,pitch,yaw
	   ackReturnData landingStatus = landing(api, flight,blockingTimeout);
	  std::cout<<"fin atterissage urgence\n";
	   done=1;
        }
	}
while(curLocalOffset.y<3 && done==0){
		int status1=moveWithVelocity(api,flight,0,speedY,0,0 ,1500, 3, 0.1);
		curPosition = api->getBroadcastData().pos;
  	curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
     if('a'==putchar(c)){
 		std::cout<<"parti pour 5 secondes stationnaires";           
		int ackStat=moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0);	   
	  	usleep(5000000);
	   	std::cout<<"fin des 5 secondes";
	   	done=1;
        }
        else if('q'==putchar(c)){
           int ackStat=moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0); //roll,pitch,yaw
	   ackReturnData landingStatus = landing(api, flight,blockingTimeout);
	  std::cout<<"fin atterissage urgence\n";
	   done=1;
        }
	}
 moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0);
 while(curLocalOffset.x>0 && done==0){
		int status1=moveWithVelocity(api,flight,-speedX,0,0,0 ,1500, 3, 0.1);
		curPosition = api->getBroadcastData().pos;
  	curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
     if('a'==putchar(c)){
 		std::cout<<"parti pour 5 secondes stationnaires";           
		int ackStat=moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0);   
	  	usleep(5000000);
	   	std::cout<<"fin des 5 secondes";
	   	done=1;
        }
        else if('q'==putchar(c)){
           int ackStat=moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0);
	   ackReturnData landingStatus = landing(api, flight,blockingTimeout);
	  std::cout<<"fin atterissage urgence\n";
	   done=1;
        }
	}
while(curLocalOffset.y>0 && done==0){
		int status1=moveWithVelocity(api,flight,0,-speedY,0,0 ,1500, 3, 0.1);
		curPosition = api->getBroadcastData().pos;
  	curEuler = Flight::toEulerAngle(api->getBroadcastData().q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
     if('a'==putchar(c)){
 		std::cout<<"parti pour 5 secondes stationnaires";           
		int ackStat=moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0);   
	  	usleep(5000000);
	   	std::cout<<"fin des 5 secondes";
	   	done=1;
        }
        else if('q'==putchar(c)){
           int ackStat=moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0);
	   ackReturnData landingStatus = landing(api, flight,blockingTimeout);
	  std::cout<<"fin atterissage urgence\n";
	   done=1;
        }
	}
 moveWithVelocity(api, flight, 0, 0, 0, 0, 2000, 0,0);
std::cout << " value of done : " << done << "\n";
	std::cout << " apres le move\n";
      //! Do aircraft control - Waypoint example. 
     // wayPointMissionExample(api, waypointObj,blockingTimeout);

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
  myfile.close();
  return 0;
}


/*



void *LinuxThread::save_data (void *param) {
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

