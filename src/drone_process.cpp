/*!*******************************************************************************************
 *  \file       drone_process.cpp
 *  \brief      DroneProcess implementation file.
 *  \details    This file implements the DroneProcess class. 
 *  \authors    Enrique Ortiz, Yolanda de la Hoz, Martin Molina, David Palacios
 *  \copyright  Copyright 2015 UPM. All right reserved. Released under license BSD-3.
 ********************************************************************************************/
#include "drone_process.h"

DroneProcess::DroneProcess()
{
  watchdog_topic = "process_alive_signal";
  error_topic = "process_error";
  char buf[32];  
  gethostname(buf,sizeof buf);  
  hostname.append(buf);
  node_handler_stopped.setCallbackQueue(&stopped_queue);//We assign the special queue to this node
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

  state_pub = node_handler_drone_process.advertise<droneMsgsROS::AliveSignal>(watchdog_topic, 10);
  error_pub = node_handler_drone_process.advertise<droneMsgsROS::ProcessError>(error_topic, 10);
  

  //ros::AdvertiseServiceOptions ops = ros::AdvertiseServiceOptions::create<std_msgs::String>(ros::this_node::getName()+"/start", &DroneProcess::startServCall, ros::VoidPtr(), &string_queue);
  // subscribe
  //ros::Subscriber sub2 = n.subscribe(ops);

  //MULTIPLES NODEHANDLERS O SOLO UNO
  //No deberian ser accesibles desde la cola global
  recoverServerSrv=node_handler_stopped.advertiseService(ros::this_node::getName()+"/recover",&DroneProcess::recoverServCall,this);
  stopServerSrv=node_handler_drone_process.advertiseService(ros::this_node::getName()+"/stop",&DroneProcess::stopServCall,this);
  startServerSrv=node_handler_stopped.advertiseService(ros::this_node::getName()+"/start",&DroneProcess::startServCall,this);

  ros::AsyncSpinner async_spinner(1,&stopped_queue); // Use 1 threads
  async_spinner.start();
  next=true;
  pthread_create( &t1, NULL, &DroneProcess::threadRun,this);
  ownOpen();
}

void DroneProcess::start()
{
  next=true;
  setState(Running);
  //ros::getGlobalCallbackQueue()->enable();
  //ownStart();
}

void DroneProcess::recover()
{
  setState(Recovering);
  ownRecover();
}

void DroneProcess::stop()
{
  next=false;
  setState(Sleeping);
  //ros::g_ok=false;
  //boost::this_thread::sleep_for(boost::chrono::seconds{10});
  //std::cout << "despierto" << std::endl;

  //ros::getGlobalCallbackQueue()->disable();
  //ros::getGlobalCallbackQueue()->clear();
  //node_handler_drone_process.getCallbackQueue()->disable();

  //stopServerSrv.shutdown();
  //stopServerSrv=node_handler_drone_process.advertiseService(ros::this_node::getName()+"/stop",&DroneProcess::stopEmptyServCall,this);
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
    //ros::spinOnce();
    //std::cout << "bucle1" << std::endl;
    notifyState();
    r.sleep();
  }
}

bool DroneProcess::recoverServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  std::cout << "recover" << std::endl;
  recover();
  return true;
}

bool DroneProcess::stopServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  std::cout << "stop" << std::endl;
  stop();
  return true;
}

bool DroneProcess::startServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  std::cout << "start" << std::endl;
  start();
  return true;
}

bool DroneProcess::recoverEmptyServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  std::cout << "Empty recover" << std::endl;
  recover();
  return true;
}

bool DroneProcess::stopEmptyServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  std::cout << "Empty stop" << std::endl;
  stop();
  return true;
}

bool DroneProcess::startEmptyServCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
  std::cout << "Empty start" << std::endl;
  start();
  return true;
}

//TODO
void DroneProcess::spin()
{
  //std::cout << "Entrando a spin" << std::endl;
  //ros::NodeHandle n;
  //n.ok()
  while(ros::ok()&&next)
  {
    std::cout << "spin" << std::endl;
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1f));
  }
}
