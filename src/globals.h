//#include "mavros_msgs/State.h"
#include "std_msgs/UInt8.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
//#include <mavros_msgs/CommandBool.h>
//#include <mavros_msgs/SetMode.h>
#include "std_msgs/Float64.h"
#include "HelperFunctions/helper.h"
#include "HelperFunctions/QuatRotEuler.h"
#include "px4_control/PVA.h"

#include "structs.h"

#include "dji_sdk/dji_sdk.h"

#include <dji_sdk/Activation.h>
//#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
//#include <dji_sdk/MissionWpAction.h>
//#include <dji_sdk/MissionHpAction.h>
//#include <dji_sdk/MissionWpUpload.h>
//#include <dji_sdk/MissionHpUpload.h>
//#include <dji_sdk/MissionHpUpdateRadius.h>
//#include <dji_sdk/MissionHpUpdateYawRate.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/DroneTaskControl.h>
//#include <dji_sdk/SendMobileData.h>
//#include <dji_sdk/QueryDroneVersion.h>



// All global variables within the code
extern PVA_structure PVA_ref;
//extern mavros_msgs::State PX4state;
extern std_msgs::UInt8 PX4state; // use dji mode
extern nav_msgs::Odometry odom;
extern joyStruct joy;
extern std::string joyDriver;
extern px4_control::PVA PVA_Ros;
extern mutexStruct mutexes;
extern joyEventList joyEvents;
extern syncEventList syncEvents;
extern StateMachine FSM;
extern int threadCount;
extern PID_3DOF PosPID;
extern PosControlParam ControlParam;

