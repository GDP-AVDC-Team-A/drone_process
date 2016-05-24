#!/usr/bin/env python
#######################################################################################################################
#  @file      drone_process.py
#  @brief     DroneProcess python file
#  @authors   Angel Luis Gonzalez Lopez, Marcos Bernal
#  @copyright Copyright 2016 Universidad Politecnica de Madrid (UPM) 
# 
#      This program is free software: you can redistribute it and/or modify 
#      it under the terms of the GNU General Public License as published by 
#      the Free Software Foundation, either version 3 of the License, or 
#      (at your option) any later version. 
#    
#      This program is distributed in the hope that it will be useful, 
#      but WITHOUT ANY WARRANTY; without even the implied warranty of 
#      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
#      GNU General Public License for more details. 
#    
#      You should have received a copy of the GNU General Public License 
#      along with this program. If not, see http://www.gnu.org/licenses/. 
#######################################################################################################################
import threading
import rospy

from cvg_stack_msgs.msg import ProcessState
from cvg_stack_msgs.msg import ProcessError
from std_msgs.msg import Header


#######################################################################################################################             
## @class   State
#  @brief   This enum defines all posible DroneProcess states that can be sent to the supervisor.
#######################################################################################################################             
class State(object):
  ## State that refers DroneProcess is initializing
  Initializing = 0
  ## State that refers DroneProcess is running on a normal execution
  Running = 1
  ## State that refers DroneProcess has finished its task and is waiting for another round (time dependence)
  Sleeping = 2
  ## State that refers DroneProcess hasnt finished and needs a critical data to continue its task (data dependence)
  Waiting = 3 
  ## State that refers DroneProcess finish its execution and has to close all its dependences 
  Stopping = 4  
  ## State that refers DroneProcess has an error and has to be reseted the process
  Reseting = 5 

#######################################################################################################################             
## @class   Error
#  @brief   This enum defines all posible DroneProcess errors that can be sent to the supervisor.
#######################################################################################################################             
class Error(object):
  ## Error that refers an unavoidable stop
  UnexpectedNodeStop = 0
  ## Error that refers an unavoidable stop without any response
  FrozenNode = 1
  ## Error that refers DroneProcess cant continue due to invalid incoming data
  InvalidInputData = 2
  ## Error that refers DroneProcess has found an error but it can be restored by itself
  SafeguardRecoverableError = 3
  ## Error that refers DroneProcess has found an unavoidable error and it cant continue
  SafeguardFatalError = 4 
  

#######################################################################################################################
## @class   DroneProcess
#  @brief   This is the core class that every ROS node has to inherit ONLY Python Language.
#  @details The DroneProcess base class adds the following functionalities to the derived
#     classes that inherits it:\n
#     - Declaration of all the states a ROS node can be at.
#     - Creation of a signal sending thread: By deriving this class the node will
#       create a thread with the only purpose of sending its state to a supervisor,
#       that will be hearing at the 'State' topic.
#     - Declaration of methods that the derived class will have to implement in order to
#       add the desired functionality to the ROS node.
#######################################################################################################################             
class DroneProcess(object):
  
  #####################################################################################################################             
  ## @brief   DroneProcess Constructor
  #  @details This method initializes the node and defines the variables used in the process 
  #  @param   node_name Name of this node. Type: String
  #####################################################################################################################             
  def __init__(self, node_name):

    ## ROS publisher handler used to send state messages.
    # Type: rospy.Publisher() instance
    self.state_pub = None

    ## ROS publisher handler used to send error messages.
    # Type: rospy.Publisher() instance
    self.error_pub = None

    ## Message of type state.
    # Type: cvg_stack_msgs.ProcessState() instance
    self.state_message = ProcessState()

    ## Attribute storing topic name to contact Supervisor.
    # Type: String
    self.watchdog_topic = "watchdog_topic";

    ## Attribute storing topic name to send errors to the Supervisor.
    # Type: String
    self.error_topic = "error_topic"

    ## Attribute storing current state of the process.
    # Type: drone_process.State() instance
    self.current_state = None

    ## Attribute storing the drone on which is executing the process.
    # Type: int
    self.drone_id = 0

    ## Attribute storing the compouter on which the process is executing.
    # Type: String
    self.hostname = ""

    ## Attribute storing the control to manage the thread that sends information to the supervision system 
    # Type: threading.Thread
    self.thread_controller = None
    
    # Start Node
    rospy.init_node(node_name)



  ##################################################################################################################### 
  ## @brief   Process initializer
  #  @details This method initializes the elements necesaries in the execution of the process 
  #####################################################################################################################
  def initialize(self):
    self._commonInitialize()
    self.ownInitialize();



  ##################################################################################################################### 
  ## @brief   Run drone process
  #  @details This method runs the node execution
  #####################################################################################################################
  def run(self):
    self._commonRun();
    self.ownRun();


  ##################################################################################################################### 
  ## @brief   Reset drone process
  #  @details This method resets the node execution
  #####################################################################################################################
  def reset(self):
    self._commonReset();
    self.ownReset();


  ##################################################################################################################### 
  ## @brief   Stop drone process
  #  @details This method stops the node execution
  #####################################################################################################################
  def stop(self):
    self.setState(State.Stopping)


  ##################################################################################################################### 
  ## @brief   Sleep drone process
  #  @details This method changes the node state to sleep
  #####################################################################################################################
  def sleep(self):
    self.setState(State.Sleeping)


  ##################################################################################################################### 
  ## @brief   Change drone process state
  #  @details This method changes the drone state to a new state obtained as a param
  #  @param   new_state New state for the process. Type: integer
  #####################################################################################################################
  def setState(self, new_state):
    if State.FirstValue < new_state and new_state < State.LastValue:
      self.current_state = new_state
      self.notifyState(State.Stopping)


  ##################################################################################################################### 
  ## @brief     Drone process state getter
  #  @details   This method returns current drone process state
  #####################################################################################################################
  def getState(self):
    return self.current_state


  ##################################################################################################################### 
  ## @brief     Drone process notifier
  #  @details   This method sends current or a new state throught watchdog_topic topic
  #  @param     current_state State to send. Type: integer
  #####################################################################################################################
  def notifyState(self, current_state = None):
    if current_state == None:
      self.state_message.state = self.getState()
    else:
      self.state_message.state = current_state

    self.state_message.node_name = rospy.get_name()
    self.state_pub.publish(self.state_message)


  ##################################################################################################################### 
  ## @brief     Error notifier
  #  @details   This method sends an error
  #  @param     error_type    Type of error Type: integer
  #  @param     reference_code  Id of error. Type: integer
  #  @param     location    Place where error happens. Type: string
  #  @param     description   Description of the error. Type: string
  #####################################################################################################################
  def notifyError(self, error_type, reference_code, location, description):
    error_msg = ProcessError()
    error_msg.Header.stamp = rospy.Time.now()
    error_msg.error_type.value = error_type
    error_message.ns = rospy.get_namespace()
    error_message.hostname = self.hostname
    error_message.node_name = rospy.get_name()
    error_message.location = location
    error_message.description = description;
    error_message.reference_code = reference_code;

    self.error_pub.publish(error_message);


  ##################################################################################################################### 
  ## @brief     Initializer
  #  @details   This method sets current state to Initializing and initializes the publishers and the threads
  #####################################################################################################################
  def _commonInitialize(self):
    self.current_state = State.Initializing
    self.state_pub = rospy.Publisher(self.watchdog_topic, ProcessState, queue_size=10)
    self.error_pub = rospy.Publisher(self.error_topic, ProcessError, queue_size=10)

    self.thread_controller = threading.Thread(target=self.threadRun)
    self.thread_controller.start()
    

  ##################################################################################################################### 
  ## @brief     Run method
  #  @details   This method changes the drone state to running
  #####################################################################################################################
  def _commonRun(self):
    self.setState(State.Running)


  ##################################################################################################################### 
  ## @brief     Reset method
  #  @details   This method changes the drone state to reset
  #####################################################################################################################
  def _commonReset(self):
    setState(State.Reseting)


  ##################################################################################################################### 
  ## @brief     Thread main method
  #  @details   This method sends the state of the node
  #####################################################################################################################
  def threadRun(self):
    r = rospy.Rate(1)
    while not rospy.is_shutdown() and not self.current_state == State.Stopping:
      self.notifyState()
      r.sleep()
    
    if not self.current_state == State.Stopping:
      self.stop()



  ##################################################################################################################### 
  ## @brief     Node initializes
  #  @details   All functions starting with 'own' has to be implemented at the derived class.
  #     This function is executed after commonInitialize() and should set up everything that the node needs to execute.
  #####################################################################################################################
  def ownInitialize(self):
    pass


  ##################################################################################################################### 
  ## @brief     Node run
  #  @details   This method runs the node execution. This method must be implemented in each class
  #####################################################################################################################
  def ownRun(self):
    pass


  ##################################################################################################################### 
  ## @brief     Node reset
  #  @details   All functions starting with 'own' has to be implemented at the derived class.
  #       This function is executed after commonReset(), and it's purpose is to reset all the parameters
  #       he developer considers necesary when needed.
  #####################################################################################################################
  def ownReset(self):
    pass
