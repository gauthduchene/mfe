/*! @file LinuxThread.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK linux example
 *  
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

//! To Implement

 
#include "LinuxThread.h"
#include "../../sample/Linux/Blocking/inc/thread.h"
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
Flight* flight2;
LinuxThread::LinuxThread()
{
    api = 0;
    type = 0;
}

LinuxThread::LinuxThread(CoreAPI *API,Flight* FLIGHT, int Type)
{
  api = API;
  flight2=FLIGHT;
  type = Type;
  api->stopCond = false;

}

bool LinuxThread::createThread()
{
  int ret = -1;
  std::string infoStr;

  /* Initialize and set thread detached attribute */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  if(1 == type)
  {
    ret = pthread_create(&threadID, NULL, send_call, (void*)api);
    infoStr = "sendPoll";
  }
  else if(2 == type)
  {
    ret = pthread_create(&threadID, NULL, read_call, (void*)api);
    infoStr = "readPoll";
  }
  
  else if (3==type)
  {
    ret = pthread_create(&threadID, NULL, callback_call, (void*)api);
    infoStr = "callback";
  }
  
    else if (4==type)
  {
    ret = pthread_create(&threadID, NULL, save_data, (void*)api);
    infoStr = "savePoll";
  }
  
    else if (5==type)
  {
    ret = pthread_create(&threadID, NULL, key_call, (void*)api);
    infoStr = "keyPoll";
  }
  
  
  else
  {
    infoStr = "error type number";
  }

  if(0 != ret)
  {
    API_LOG(api->getDriver(), ERROR_LOG, "fail to create thread for %s!\n",
    infoStr.c_str());
    return false;
  }

  ret = pthread_setname_np(threadID, infoStr.c_str());
  if(0 != ret)
  {
    API_LOG(api->getDriver(), ERROR_LOG, "fail to set thread name for %s!\n",
            infoStr.c_str());
    return false;
  }
  return true;
}

int LinuxThread::stopThread()
{
  int ret = -1;
  void *status;
  api->stopCond = true;

  /* Free attribute and wait for the other threads */
  pthread_attr_destroy(&attr);

  ret = pthread_join(threadID, &status);

  if(ret)
  {
    // Return error code
    return ret;
  }

  API_LOG(api->getDriver(), DEBUG_LOG, "Main: completed join with thread\n");
  return 0;
}

void *LinuxThread::send_call(void *param)
{
  while(true)
  {
    ((CoreAPI*)param)->sendPoll();
  }
}

void *LinuxThread::read_call(void *param)
{
  while(!((CoreAPI*)param)->stopCond)
  {
    ((CoreAPI*)param)->readPoll();
  }
}

void *LinuxThread::callback_call(void *param)
{
  while(true)
  {
    ((CoreAPI*)param)->callbackPoll((CoreAPI*)param);
  }
}

void *LinuxThread::save_data(void *param)
{
  while(true)
  {
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
  curPosition = flight2 -> getPosition();

	originPosition = curPosition;
 localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);

	//myfile.open ("test.csv");
	//myfile << "Quaternion q0,Quaternion q1,quaternion q2,quaternion q3,velocity x,velocity y, velocity z,latitude,longitude,altitude,height, acceleration x,acceleration y, acceleration z, mag x, mag y, mag z,Yaw,Roll,Pitch,\n";
pos=flight2 -> getPosition();
    while (i<100){
        q=flight2   -> getQuaternion();
        v=flight2   -> getVelocity();
        pos=flight2 -> getPosition();
	acc=flight2 -> getAcceleration();
	ya=flight2  -> getYawRate();
	mag=flight2 -> getMagnet();
        yaw=flight2 ->getYaw();
	roll=flight2 ->getRoll();
        pitch=flight2 ->getPitch();
        curPosition =flight2 -> getPosition();
curEuler = Flight::toEulerAngle(q);
    localOffsetFromGpsOffset(curLocalOffset, &curPosition, &originPosition);
         std::cout<< "la valeur de la position dans le thread:" << curLocalOffset.x << "\n" ;
	//gps=flight -> 
	// rtk
	//rc	

	//myfile <<q.q0  <<","<< q.q1 <<","<< q.q2 <<","<< q.q3 <<","<< v.x <<","<< v.y <<","<< v.z <<","<< pos.latitude <<","<< pos.longitude <<","<< pos.altitude <<","<< pos.height <<","<< acc.x <<","<< acc.y <<","<< acc.z <<","<< mag.x <<","<< mag.y <<","<< mag.z <<","<<yaw <<","<<roll<<","<<pitch<< ","<< curLocalOffset.x << ","<< curLocalOffset.y << ","<<curLocalOffset.z <<",\n";

	i=i+1;
	usleep(500000);
  }
}
}
void *LinuxThread::key_call(void *param)
{
  while(true)
  {
    
  }
}
