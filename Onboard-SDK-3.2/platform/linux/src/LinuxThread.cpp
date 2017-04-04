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

//UHD Library Headers

#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <complex>


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

int argc;
char* argv[3];
Flight* flight2;
namespace po = boost::program_options;
using namespace std;
ofstream myfile;
float* secondPointer;
int* numberTest;

LinuxThread::LinuxThread()
{
    api = 0;
    type = 0;
}

LinuxThread::LinuxThread(CoreAPI *API,Flight* FLIGHT,float* pointerRadio,int* number,int argc2, char* argv2[], int Type)
{
  api = API;
  flight2=FLIGHT;
  type = Type;
  api->stopCond = false;
  argc = argc2;
  argv[0] = argv2[0];
  argv[1] = argv2[2];
  argv[2] = argv2[3];
  secondPointer=pointerRadio;
  numberTest=number;
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
  
   else if (6==type)
  {
    ret = pthread_create(&threadID, NULL, radio, (void*)api);
    infoStr = "radio";
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

	myfile.open ("test.csv");
	myfile << "Quaternion q0,Quaternion q1,quaternion q2,quaternion q3,velocity x,velocity y, velocity z,latitude,longitude,altitude,height, acceleration x,acceleration y, acceleration z, mag x, mag y, mag z,Yaw,Roll,Pitch,Posx, PosY, PosZ,number test, radio value,\n";
pos=flight2 -> getPosition();
    while (1){
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
        // std::cout<< "la valeur de la position dans le thread:" << curLocalOffset.x << "\n" ;
	//gps=flight -> 
	// rtk
	//rc	

	myfile <<q.q0  <<","<< q.q1 <<","<< q.q2 <<","<< q.q3 <<","<< v.x <<","<< v.y <<","<< v.z <<","<< pos.latitude <<","<< pos.longitude <<","<< pos.altitude <<","<< pos.height <<","<< acc.x <<","<< acc.y <<","<< acc.z <<","<< mag.x <<","<< mag.y <<","<< mag.z <<","<<yaw <<","<<roll<<","<<pitch<< ","<< curLocalOffset.x << ","<< curLocalOffset.y << ","<<curLocalOffset.z <<","<<*numberTest <<","<<*secondPointer<< ",\n";

	i=i+1;
	usleep(5000);
  }
}
}
void *LinuxThread::key_call(void *param)
{
  while(true)
  {
    
  }
}

void *LinuxThread::radio(void *param)
{ 
  while(true)
  {
	uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args;
    std::string wire;
    double seconds_in_future;
    size_t total_num_samps;
    size_t spb;
    double rate;
    std::string channel_list;
    double freq;
    double gain;
    double norm;
    *secondPointer=12545;
    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("wire", po::value<std::string>(&wire)->default_value(""), "the over the wire type, sc16, sc8, etc")
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(1000), "total number of samples to receive")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming samples")
        ("dilv", "specify to disable inner-loop verbose")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("freq", po::value<double>(&freq)->default_value(632e6), "RF center frequency in Hz")
        ("spb", po::value<size_t>(&spb)->default_value(1000), "samples per buffer")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX Timed Samples %s") % desc << std::endl;

    }

    bool verbose = vm.count("dilv") == 0;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

   //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = boost::lexical_cast<int>(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels() or chan >= usrp->get_rx_num_channels()){
            throw std::runtime_error("Invalid channel(s) specified.");
        }
        else channel_nums.push_back(boost::lexical_cast<int>(channel_strings[ch]));
    }

    //set the rx sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));
    
     //set the center frequency
    if (vm.count("freq")) { //with default of 0.0 this will always be true
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_rx_freq(tune_request);
        std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq()/1e6) << std::endl << std::endl;
    }
    
     //set the rf gain
    if (vm.count("gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
        usrp->set_rx_gain(gain);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain() << std::endl << std::endl;
    }

    //create a receive streamer
    uhd::stream_args_t stream_args("fc32", wire); //complex floats
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
while(1) { 
    //setup streaming
  //  std::cout << std::endl;
  //  std::cout << boost::format(
  //      "Begin streaming %u samples, %f seconds in the future..."
 //   ) % total_num_samps % seconds_in_future << std::endl;
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = total_num_samps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
    rx_stream->issue_stream_cmd(stream_cmd);

    //meta-data will be filled in by recv()
    uhd::rx_metadata_t md;

    //allocate buffer to receive with samples
    std::vector<std::complex<float> > buff(rx_stream->get_max_num_samps());
    std::vector<void *> buffs;
    for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
        buffs.push_back(&buff.front()); //same buffer for each channel

    //the first call to recv() will block this many seconds before receiving
    double timeout = seconds_in_future + 0.1; //timeout (delay before receive + padding)
	float avg =0;
    size_t num_acc_samps = 0; //number of accumulated samples
    while(num_acc_samps < total_num_samps){
        //receive a single packet
        size_t num_rx_samps = rx_stream->recv(
            buffs, buff.size(), md, timeout, true
        );
		for(int j=0; j<1000;j++){
			norm=abs(buff[j]);
		    avg=avg+norm;
		}
		avg=avg/1000;
		//std::cout<<" valeur de la moyenne   " << avg<<std::endl;
        //use a small timeout for subsequent packets
        timeout = 0.1;
		*secondPointer=avg;
        //handle the error code
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            throw std::runtime_error(str(boost::format(
                "Receiver error %s"
            ) % md.strerror()));
        }

       // if(verbose) std::cout << boost::format(
       //     "Received packet: %u samples, %u full secs, %f frac secs"
       // ) % num_rx_samps % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;

        num_acc_samps += num_rx_samps;
    }

    if (num_acc_samps < total_num_samps) std::cerr << "Receive timeout before all samples received..." << std::endl;

    //finished
    //std::cout << std::endl << "Done!" << std::endl << std::endl;
    seconds_in_future = seconds_in_future +0.1;
}
  
}

}

