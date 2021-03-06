#ifndef _H_STRUCTS_
#define _H_STRUCTS_

#include "ros/ros.h"
#include <pthread.h>
#include "pevents/pevents.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include "std_msgs/Float64.h"
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"

//Structure used in this code to pass PVA references
struct PVA_structure{
  geometry_msgs::PoseStamped  Pos;
  geometry_msgs::TwistStamped Vel;
  geometry_msgs::AccelStamped Acc;
  std_msgs::Float64 thrustRef;
};

//Incoming data from joysticks
struct joyStruct{
	int seq; //Sequence of the message
	int buttonA;
	int buttonB;
	int buttonX;
	int buttonY;
	int buttonR1;
	int buttonL1;
	int buttonSelect;
	int buttonStart;
	int buttonLeft;
	int buttonRight;
	int buttonUp;
	int buttonDown;

	double LstickHor;
	double LstickVer;
	double RstickHor;
	double RstickVer;
	double L2;
	double R2;
};

//Event handlers that are triggered when joystick buttons are pushed
struct joyEventList{
	neosmart::neosmart_event_t buttonA;
	neosmart::neosmart_event_t buttonB;
	neosmart::neosmart_event_t buttonX;
	neosmart::neosmart_event_t buttonY;
	neosmart::neosmart_event_t buttonR1;
	neosmart::neosmart_event_t buttonL1;
	neosmart::neosmart_event_t buttonSelect;
	neosmart::neosmart_event_t buttonStart;
	neosmart::neosmart_event_t buttonLeft;
	neosmart::neosmart_event_t buttonRight;
	neosmart::neosmart_event_t buttonUp;
	neosmart::neosmart_event_t buttonDown;
};

//Events for thread synchronization
struct syncEventList{
	neosmart::neosmart_event_t Timeout;     //Always zero event
	neosmart::neosmart_event_t Terminate; 	//Program termination event
	neosmart::neosmart_event_t Joy_trigger; //triggers joystick thread
	neosmart::neosmart_event_t CommPub_trigger; //triggers joystick thread
};

//Mutexes for safely sharing data between threads
struct mutexStruct{
	pthread_mutex_t PVAref;
	pthread_mutex_t PVA_ros;
	pthread_mutex_t PX4state;
	pthread_mutex_t odom;
	pthread_mutex_t joy;
	pthread_mutex_t joyEvents;
	pthread_mutex_t FSM;
	pthread_mutex_t threadCount;
	pthread_mutex_t PID_Pos;
	pthread_mutex_t PID_Param;
};

//Data for state machine
struct StateMachine{
	int MODE_DISARM;
	int MODE_ATTITUDE;
	int MODE_POSITION_JOY;
	int MODE_POSITION_ROS;

	int POS_CONTROL_LOCAL;
	//int POS_CONTROL_PX4;

	int POS_REF_WORLD;
	int POS_REF_BODY;

	int State;
	int PosControlMode;
	int PosRefMode;
};

//Structure for a PID with 3 degrees-of-freedom
struct PID_3DOF{
	Eigen::Vector3d e_prop;
	Eigen::Vector3d e_deriv;
	Eigen::Vector3d e_integ;
	Eigen::Vector3d feedForward;
	Eigen::Vector3d K_p;
	Eigen::Vector3d K_i;
	Eigen::Vector3d K_d;
	Eigen::Vector3d maxInteg;
};

//Quadcopter parameters
struct PosControlParam{
	double mass;
	double gz;
	double thrustRatio;
};

typedef struct ServiceAck{
  bool result;
  int cmd_set;
  int cmd_id;
  unsigned int ack_data;
  ServiceAck(bool res, int set, int id, unsigned int ack):
      result(res), cmd_set(set), cmd_id(id), ack_data(ack) {};
  ServiceAck() {};
}ServiceAck;


//Initialize PVA structure
void initializePVA(PVA_structure &PVA);

//Initialize values for the joy structure
void initializeJoy(joyStruct &Joy);

//Initialize event handles
void initializeEvents(joyEventList &JoyEvents, syncEventList &SyncEvents);

//Destroy all event handles
void destroyEvents(joyEventList &JoyEvents, syncEventList &SyncEvents);

//Initialize mutex handles
void initializeMutexes(mutexStruct &mutexes);

//Destroy all mutex handles
void destroyMutexes(mutexStruct &mutexes);

//Set initial value for the state machine
void initializeStateMachine(StateMachine &FSM);

//Print states from the state machine
void printCurrentState(StateMachine FSM);


#endif

