/*!*******************************************************************************************
 *  \file       drone_process.cpp
 *  \brief      DroneProcess implementation file.
 *  \details    This file implements the DroneProcess class. 
 *  \authors    Enrique Ortiz, Yolanda de la Hoz, Martin Molina, David Palacios
 *  \copyright  Copyright 2016 Universidad Politecnica de Madrid (UPM) 
 *
 *     This program is free software: you can redistribute it and/or modify 
 *     it under the terms of the GNU General Public License as published by 
 *     the Free Software Foundation, either version 3 of the License, or 
 *     (at your option) any later version. 
 *   
 *     This program is distributed in the hope that it will be useful, 
 *     but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 *     GNU General Public License for more details. 
 *   
 *     You should have received a copy of the GNU General Public License 
 *     along with this program. If not, see http://www.gnu.org/licenses/. 
 ********************************************************************************/

#include "drone_process.h"

DroneProcess::DroneProcess()
{
  watchdog_topic = "process_alive_signal";
  error_topic = "process_error";
  char buf[32];  
  gethostname(buf,sizeof buf);  
  hostname.append(buf);
  beginning=true;

  current_state = Created; //This State is not going to be sent. It will we significative if in the future we implement a state check in every function.
}

DroneProcess::~DroneProcess()
{
  pthread_cancel(t1);
}

void DroneProcess::setUp()
{
  //TODO check if shutdown closes this
  state_pub = node_handler_drone_process.advertise<droneMsgsROS::AliveSignal>(watchdog_topic, 10);
  error_pub = node_handler_drone_process.advertise<droneMsgsROS::ProcessError>(error_topic, 10);
  
  recoverServerSrv=node_handler_drone_process.advertiseService(ros::this_node::getName()+"/recover",&DroneProcess::recoverServCall,this);
  stopServerSrv=node_handler_drone_process.advertiseService(ros::this_node::getName()+"/stop",&DroneProcess::stopServCall,this);
  startServerSrv=node_handler_drone_process.advertiseService(ros::this_node::getName()+"/start",&DroneProcess::startServCall,this);

  pthread_create( &t1, NULL, &DroneProcess::threadRun,this);
  ownSetUp();
  setState(ReadyToStart);
}

void DroneProcess::start()
{
  setState(Running);
  ownStart();
  if(beginning)
  {
    beginning=false;
    ownRun();
  }
}

//TODO, see what we will do with this function/state
void DroneProcess::recover()
{
  setState(Recovering);
  ownRecover();
}

void DroneProcess::stop()
{
  setState(ReadyToStart);
  n.shutdown();
}

std::string DroneProcess::stateToString(State state)
{
  static const char* statesArray[] =
    { 
      "FirstValue",
      "Created",
      "ReadyToStart",
      "Running",
      "Paused",
      "Recovering",
      "UnexpectedState",
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



//TODO
//Comprobar estados al recibir llamadas
bool DroneProcess::recoverServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  recover();
  return true;
}

bool DroneProcess::stopServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  ROS_INFO("Start deeeeeeee");
  //ROS_DEBUG("stopppppppppp");
  //ROS_WARN("AaaaaaaaaaAAAAAAAAAAAAAA");
  //ROS_ERROR("ERORRRrrrrrrrrrrrrrrrrrrr");
  //ROS_FATAL("fhdjklfhldjkhfldfjldbgfjkdgfjk");
  //ROS_INFO_NAMED("Start","mensajeeeeejikjkjiefsfssfsdsfsdsd");
  ROS_INFO_COND(current_state==Running,"Paramos");
  if(current_state==Running)
  {
    stop();
    return true;
  }
  else
  {
    return false;
  }
}

bool DroneProcess::startServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  start();
  return true;
}

void DroneProcess::syncRun()
{
  if(current_state==Running)
    ownSyncRun();
}