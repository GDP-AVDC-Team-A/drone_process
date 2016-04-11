/*!********************************************************************************************************************
 *  \file       drone_process.h
 *  \brief      DroneProcess definition file.
 *  \details    This file contains the DroneProcess declaration. To obtain more information about
 *              it's definition consult the drone_process.cpp file.
 *  \authors    Enrique Ortiz, Yolanda de la Hoz, Martin Molina
 *  \copyright  Copyright 2016 UPM. All right reserved. Released under license BSD-3.
 *********************************************************************************************************************/

#ifndef DRONE_PROCESS
#define DRONE_PROCESS

#include <string>
#include <stdio.h>
#include <stdexcept>
#include <pthread.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <droneMsgsROS/AliveSignal.h>
#include <droneMsgsROS/ProcessError.h>

/*!********************************************************************************************************************
 *  \class      DroneProcess
 *  \brief      This is the core class that every ROS node has to inherit.
 *  \details    The DroneProcess base class adds the following functionalities to the derived
 *              classes that inherits it:\n
 *              - Declaration of all the states a ROS node can be at.
 *              - Creation of a signal sending thread: By deriving this class the node will
 *                  create a thread with the only purpose of sending its state to a PerformanceMonitor,
 *                  that will be hearing at the 'State' topic.
 *              - Declaration of methods that the derived class will have to implement in order to
 *                  add the desired functionality to the ROS node.
 * 
 *********************************************************************************************************************/
class DroneProcess
{
  
public:
  /*!******************************************************************************************************************
   *  \brief     This enum defines all posible DroneProcess states that can be sent to the PerformanceMonitor.
   *  \details   The states Started and NotStarted are here to maintain consistency with the class DroneModule.
   *             The states FirstValue and LastValue have no meaning, they are used to facilitate the implementation.  
   *******************************************************************************************************************/
  typedef enum
  {
    FirstValue,
    Opening,
    Running,
    Sleeping,
    Closing,
    Recovering,
    Started,
    NotStarted,
    LastValue
  } State;

  //! This enum defines all posible DroneProcess errors that can be sent to the PerformanceMonitor.
  typedef enum
  {
    UnexpectedProcessStop,
    InvalidInputData,
    SafeguardRecoverableError,
    SafeguardFatalError,
  } Error;

  //! Constructor. \details It needs the same arguments as the ros::init function.
  DroneProcess(int argc, char **argv);
      
  /*!******************************************************************************************************************
   * The PerformanceMonitor get's a "Stopping" state notification at object's destruction.
   *******************************************************************************************************************/
  ~DroneProcess();
    
  //!  This function calls to ownInitialize().
  void open();

  //!  This function calls to ownRun().
  void start();

  void stop();

  //!  This function calls to ownRecover().
  void recover();

   /*!*****************************************************************************************************************
   * \details If the node has an already defined state (Waiting, Running...) returns
   * the state as an Integer, if not it returns -1 to indicate the current state is undefined.
   * \return Void function
   *******************************************************************************************************************/
  State getState();

  /*!******************************************************************************************************************
   * \details The function accepts one of the already defined states to modify the 'curent_state' attribute. 
   * It also sends and alive message to the PerformanceMonitor indicating the new state of the node.
   * \param   new_state The new state the process is going to have.
   * \return  The current state of the process.
   *******************************************************************************************************************/
  void setState(State new_state);

  /*!******************************************************************************************************************
   * \brief Send a DroneProcess.error to the PerformanceMonitor
   * \details This function is a first aproach at error handling. The error comunication between nodes
   * is performed by a two-part message. The first part indicates the type of message we are sending 
   * (info, warning, error) while the second part offers a detailed description of the problem.
   * \param [in] type            The type of error we are going to send.
   * \param [in] reference_code  This is a numeric code that may be usefull during error processing.
   * \param [in] location        The location is a string that explains at which function the error occured.
   * \param [in] description     Another String for the human reader that explains in detail the error.
   *******************************************************************************************************************/
  void notifyError(Error type, int reference_code, std::string location, std::string description);

  /*!******************************************************************************************************************
   * \brief The stateToString method transforms the recieved state into a human readable String.
   * \param [in] state The received state that need to be transformed.
   * \return the state in a String form.
   *******************************************************************************************************************/
  std::string stateToString(State state);

private:

  //!  This function sends an alive message to the PerformanceMonitor indicating the current node state.
  void notifyState();

  /*!******************************************************************************************************************
   * \brief This function sends an alive message to the PerformanceMonitor indicating the current node state.
   * \param [in] state State that has to be sent to the PerformanceMonitor.
   *******************************************************************************************************************/
  void notifyState(State state);

  /*!******************************************************************************************************************
   * \brief This function has the purpose to serve as the thread execution point.
   * \param [in] argument Function which has to be executed by the thread.
   *******************************************************************************************************************/
  static void * threadRun(void * argument);

  //!  This function implements the thread's logic.
  void threadAlgorithm();

  pthread_t t1; //!< Thread handler.

  //! ROS service handler used to order a process to try to recover from some fault.
  ros::ServiceServer recoverServerSrv;
  
  //! ROS service handler used to order a process to start.
  ros::ServiceServer startServerSrv;
  
  //! ROS service handler used to order a process to stop.
  ros::ServiceServer stopServerSrv;

  /*!******************************************************************************************************************
   * \brief This ROS service set DroneProcess in recovering state.
   *
   *        TODO: THIS FUNCTION HAS TO BE REVIEWED 
   *
   * \param [in] request 
   * \param [in] response 
   *******************************************************************************************************************/ 
  bool recoverServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  
  /*!******************************************************************************************************************
   * \brief This ROS service set DroneProcess in running state.
   *
   *        TODO: THIS FUNCTION HAS TO BE REVIEWED 
   *
   * \param [in] request 
   * \param [in] response 
   *******************************************************************************************************************/ 
  bool stopServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
  
  /*!******************************************************************************************************************
   * \brief This ROS service set DroneProcess in sleeping state.
   *
   *        TODO: THIS FUNCTION HAS TO BE REVIEWED 
   *
   * \param [in] request 
   * \param [in] response 
   *******************************************************************************************************************/ 
  bool startServCall(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  ros::Publisher state_pub;            //!< ROS publisher handler used to send state messages.
  ros::Publisher error_pub;            //!< ROS publisher handler used to send error messages.

  droneMsgsROS::AliveSignal state_message; //!< Message of type state.
  
  std::string watchdog_topic;       //!< Attribute storing topic name to send alive messages to the PerformanceMonitor.
  std::string error_topic;          //!< Attribute storing topic name to send errors to the PerformanceMonitor.

protected:
  State current_state;               //!< Attribute storing current state of the process.
  std::string drone_id;              //!< Attribute storing the drone on which is executing the process.
  std::string hostname;              //!< Attribute storing the compouter on which the process is executing.

  /*!******************************************************************************************************************
   * \details All functions starting with 'own' has to be implemented at the derived class.
   * This function is executed after commonInitialize() and should set up everything that the node needs to execute.
   *******************************************************************************************************************/
  virtual void ownOpen()= 0;

  /*!******************************************************************************************************************
   * \details All functions starting with 'own' has to be implemented at the derived class.
   * This function is executed after commonRun(). It should contain the main loop of the node.
   *******************************************************************************************************************/
  virtual void ownRun()= 0;
  
  /*!******************************************************************************************************************
   * \details All functions starting with 'own' has to be implemented at the derived class.
   * This function is executed after commonRecover(), and it's purpose is to recover all the parameters
   * the developer considers necesary when needed.
   *******************************************************************************************************************/
  virtual void ownRecover()= 0;

};
#endif
