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
  //node_handler_stopped.setCallbackQueue(&supervision_queue);//We assign the special queue to this node

  //finished=false;
  started=true;
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

  //TODO see if shutdown closes this
  state_pub = node_handler_drone_process.advertise<droneMsgsROS::AliveSignal>(watchdog_topic, 10);
  error_pub = node_handler_drone_process.advertise<droneMsgsROS::ProcessError>(error_topic, 10);
  
  recoverServerSrv=node_handler_stopped.advertiseService(ros::this_node::getName()+"/recover",&DroneProcess::recoverServCall,this);
  stopServerSrv=node_handler_stopped.advertiseService(ros::this_node::getName()+"/stop",&DroneProcess::stopServCall,this);
  startServerSrv=node_handler_stopped.advertiseService(ros::this_node::getName()+"/start",&DroneProcess::startServCall,this);

  //std::cout << "creo spinner" << std::endl;
  //ros::AsyncSpinner local_spinner(1,&supervision_queue); // Uses 1 threads
  //local_spinner.start();
  
  pthread_create( &t1, NULL, &DroneProcess::threadRun,this);
  ownOpen();

  //TODO, move other place
  //TODO add another lock for finished
  /*
  while(!finished)//TODO check state
  {
    boost::mutex::scoped_lock lock(mut);
    while(!started)
    {
      std::cout << "entro a esperar" << std::endl;
      cond.wait(lock);
      std::cout << "salgo de esperar" << std::endl;
    }
    lock.unlock();
    //Add if started or if paused
    ownRun();
  }*/
}

void DroneProcess::run()
{
  setState(Running);
  ownStart();
  ownRun();
}


void DroneProcess::start()
{
  //boost::mutex::scoped_lock lock(mut);
  //ros::NodeHandle ntemp;
  //std::cout << (ntemp.ok()?"true":"false") << std::endl;
  //std::cout << (n.ok()?"true":"false") << std::endl;
  //(&n)=(&ntemp);
  //n.~NodeHandle();
  //n = ros::NodeHandle();
  //n(ntemp);
  //n=ntemp;
  //delete &n;
  //ros::NodeHandle n;
  n::NodeHandle();
  //std::cout << (n.ok()?"true":"false") << std::endl;
  //n=ntemp;
  setState(Running);
  /*started=true;
  lock.unlock();
  cond.notify_one();*/
  //ros::getGlobalCallbackQueue()->clear();
  //ros::getGlobalCallbackQueue()->enable();
  //std::cout << "empty= " << (ros::getGlobalCallbackQueue()->isEmpty()?"true":"false") << std::endl;
  //std::cout << "enable= " <<(ros::getGlobalCallbackQueue()->isEnabled()?"true":"false") << std::endl;
  //TODO, llamadas a ownStart solo cuando hemos parado
  ownStart();
}

void DroneProcess::recover()
{
  setState(Recovering);
  ownRecover();
}

void DroneProcess::stop()
{
  setState(Sleeping);
  //boost::mutex::scoped_lock lock(mut);
  //started=false;
  n.shutdown();
  //setServCall();
  //lock.unlock();
  //ros::getGlobalCallbackQueue()->disable();
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

void DroneProcess::setServCall()
{
  recoverServerSrv=node_handler_stopped.advertiseService(ros::this_node::getName()+"/recover",&DroneProcess::recoverServCall,this);
  stopServerSrv=node_handler_stopped.advertiseService(ros::this_node::getName()+"/stop",&DroneProcess::stopServCall,this);
  startServerSrv=node_handler_stopped.advertiseService(ros::this_node::getName()+"/start",&DroneProcess::startServCall,this);
}

void DroneProcess::syncRun()
{
  if(/*current_state==Running*/n.ok())
    ownSyncRun();
}