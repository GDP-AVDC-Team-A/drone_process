/*!*******************************************************************************************
 *  \file       drone_process.cpp
 *  \brief      DroneProcess implementation file.
 *  \details    This file implements the DroneProcess class. 
 *  \author     Enrique Ortiz
 *  \copyright  Copyright 2015 UPM. All right reserved. Released under license BSD-3.
 ********************************************************************************************/
#include "drone_process.h"

DroneProcess::DroneProcess(int argc, char **argv)
{
  watchdog_topic = "process_alive_signal";
  error_topic = "process_error";
  char buf[32];  
  gethostname(buf,sizeof buf);  
  hostname.append(buf);
  ros::init(argc, argv, ros::this_node::getName());
}

DroneProcess::~DroneProcess()
{
  pthread_cancel(t1);
  setState(Closing);
  notifyState(Closing);
}

void DroneProcess::open()
{
  current_state = Opening;
  ros::NodeHandle n;
  state_pub = n.advertise<droneMsgsROS::AliveSignal>(watchdog_topic, 10);
  error_pub = n.advertise<droneMsgsROS::ProcessError>(error_topic, 10);
  recoverServerSrv=n.advertiseService(ros::this_node::getName()+"/recover",&DroneProcess::recoverServCall,this);
  stopServerSrv=n.advertiseService(ros::this_node::getName()+"/stop",&DroneProcess::stopServCall,this);
  startServerSrv=n.advertiseService(ros::this_node::getName()+"/start",&DroneProcess::startServCall,this);
  pthread_create( &t1, NULL, &DroneProcess::threadRun,this);
  ownOpen();
}

void DroneProcess::start()
{
  setState(Running);
  //ownStart();
}

void DroneProcess::recover()
{
  setState(Recovering);
  ownRecover();
}

void DroneProcess::stop()
{
  setState(Sleeping);
  //ownStop();
}

std::string DroneProcess::stateToString(State state)
{
  static const char* statesArray[] =
    { 
      "FirstValue",
      "Initializing",
      "Running",
      "Sleeping",
      "Waiting",
      "Stopping",
      "Recovering",
      "Started",
      "NotStarted",
      "LastValue",
    };
    std::string result;

  if((int)state < LastValue && (int)state > FirstValue)
   result.assign(statesArray[(int)state]);

  return result;
}

DroneProcess::State DroneProcess::getState()
{
  return current_state;
}

void DroneProcess::setState(State new_state)
{
  if (new_state > FirstValue && new_state < LastValue)
  {
    current_state = new_state;
    notifyState();
  }
}

void DroneProcess::notifyState()
{
  State _current_state = getState();
  if(_current_state > 0)
  {
    state_message.header.stamp = ros::Time::now();
    state_message.hostname = hostname;
    state_message.process_name = ros::this_node::getName();
    state_message.current_state.state = (int)_current_state;
    state_pub.publish(state_message);
  }
}

void DroneProcess::notifyState(State current_state)
{
  state_message.header.stamp = ros::Time::now();
  state_message.process_name = ros::this_node::getName();
  state_message.hostname = hostname;
  state_message.current_state.state = (int)current_state;
  state_pub.publish(state_message);
}


void DroneProcess::notifyError(Error type, int reference_code, std::string function, std::string description)
{
  droneMsgsROS::ProcessError error_message;
  error_message.header.stamp = ros::Time::now();
  error_message.error_type.value = (int)type;
  error_message.hostname = hostname;
  error_message.process_name = ros::this_node::getName();
  error_message.function = function;
  error_message.description = description;
  error_message.reference_code = reference_code;
  error_pub.publish(error_message);
}

// COMMON METHODS
void * DroneProcess::threadRun(void * argument)
{
  ((DroneProcess *) argument)->threadAlgorithm();
  return NULL;
}

void DroneProcess::threadAlgorithm()
{
  ros::Rate r(1);
  while(ros::ok())
  {
    notifyState();
    r.sleep();
  }
}

bool DroneProcess::recoverServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  recover();
  return true;
}

bool DroneProcess::stopServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  stop();
  return true;
}

bool DroneProcess::startServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  start();
  return true;
}
