#include "ros/ros.h"

#include "structs.h"
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"
#include "Callbacks/callbacks.h"
#include "Threads/FSMTask.h"
#include "Threads/joyTask.h"
#include "Threads/commPub.h"
#include "Services/services.h"

#include <pthread.h>


// Global variables
PVA_structure PVA_ref;        //Joystick references
px4_control::PVA PVA_Ros;     //References from topic
//mavros_msgs::State PX4state;
std_msgs::UInt8 PX4state;
nav_msgs::Odometry odom;
joyStruct joy;
joyEventList joyEvents;
syncEventList syncEvents;
mutexStruct mutexes;
StateMachine FSM;
int threadCount = 0;
PID_3DOF PosPID;
PosControlParam ControlParam;
std::string odomTopic, joyDriver, pvaTopic;

ros::ServiceClient drone_activation_service; // dji drone activation

ServiceAck activate() {
  dji_sdk::Activation activation;
  drone_activation_service.call(activation);
  if(!activation.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set, activation.response.cmd_id);
    ROS_WARN("ack.data: %i", activation.response.ack_data);
  }
  return {activation.response.result, activation.response.cmd_set,
          activation.response.cmd_id, activation.response.ack_data};
}


int main(int argc, char **argv)
{
  //Initialize ROS
  ros::init(argc, argv, "controlPkg");
  ros::NodeHandle n;  

  //Initialize some variables
  initializePVA(PVA_ref);
  initializeJoy(joy);
  initializeEvents(joyEvents, syncEvents);
  initializeMutexes(mutexes);
  initializeStateMachine(FSM);
  initializePID3(PosPID);
  readROSparameterServer(PosPID, ControlParam);

  //Print initial state of the finite state machine
  printCurrentState(FSM);

  //Get odometry topic and joystick driver
  // TODO change to launch file parameter
  ros::param::get("px4_control_node/odomTopic", odomTopic);
  ros::param::get("px4_control_node/joyDriver", joyDriver);
  ros::param::get("px4_control_node/pvaTopic", pvaTopic);

  //Create services ------------------------------------------
  ros::ServiceServer PID_srv = n.advertiseService
    ("px4_control_node/updatePosControlParam", updatePosControlParam);
  ros::ServiceServer Param_srv = n.advertiseService
    ("px4_control_node/updateQuadParam", updateSystemParam);

  // create service client
  drone_activation_service = n.serviceClient<dji_sdk::Activation> 
    ("dji_sdk/activation");

  //Subscribers ----------------------------------------------
  //ros::Subscriber stateSub = n.subscribe("mavros/state", 10, stateCallback);
  ros::Subscriber stateSub = n.subscribe("dji_sdk/display_mode", 10, stateCallback); // use dji_sdk
  ros::Subscriber odomSub = n.subscribe(odomTopic, 10, odomCallback);
  ros::Subscriber tfSub = n.subscribe(odomTopic, 10, tfCallback);
  ros::Subscriber joySub = n.subscribe("joy", 10, joyCallback);
  ros::Subscriber PvaSub = n.subscribe(pvaTopic, 10, PVACallback);

  //Threads --------------------------------------------------
  pthread_t h_FSMThread;      //Finite state machine
  pthread_t h_joyThreadTimer; //Timer for joystick thread
  pthread_t h_joyThread;      //Joystick thread
  pthread_t h_commPubTimer;   //Timer for command publisher
  pthread_t h_commPubThread;  //Command publisher thread
  int ReturnCode;

  //Start  finite state machine
  if (ReturnCode = pthread_create(&h_FSMThread, NULL, FSMTask, NULL)){
    printf("Start State Machine failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
    threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }

  //Start joystick timer thread
  if (ReturnCode = pthread_create(&h_joyThreadTimer, NULL, joyTaskTimer, NULL)){
    printf("Start joystick timer thread failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
    threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }


  //Start joystick thread
  if (ReturnCode = pthread_create(&h_joyThread, NULL, joyTask, NULL)){
    printf("Start joystick thread failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
    threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }

  //Start command publisher timer thread
  if (ReturnCode = pthread_create(&h_commPubTimer, NULL, commPubTimer, NULL)){
    printf("Start command publisher timer thread failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
    threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }


  //Start command publisher thread
  if (ReturnCode = pthread_create(&h_commPubThread, NULL, commPubTask, NULL)){
    printf("Start command publisher thread failed; return code from pthread_create() is %d\n", ReturnCode);
    exit(-1);
  }
  else{
    pthread_mutex_lock(&mutexes.threadCount);
    threadCount += 1;
    pthread_mutex_unlock(&mutexes.threadCount);
  }

  // activate dji drone
  ServiceAck service_ack;
  service_ack = activate();
  if (service_ack.result) {
    ROS_INFO("Activated successfully");
  } else {
    ROS_WARN("Failed activation");
  }

  //Start loop ----------------------------------------------------
  ros::Rate loop_rate(500);

  int localThreadCount;
  while (ros::ok())
  {

    //Check if all threads were terminated
    pthread_mutex_lock(&mutexes.threadCount);
    localThreadCount = threadCount;
    pthread_mutex_unlock(&mutexes.threadCount);
    if(localThreadCount == 0){
      break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  //Terminate program ---------------------------------------------
  destroyEvents(joyEvents, syncEvents);
  destroyMutexes(mutexes);

  ROS_INFO("Process Ended with Success!");


  return 0;
}


