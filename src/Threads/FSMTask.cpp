#include "FSMTask.h"

using namespace DJI::OSDK;

void *FSMTask(void *threadID) {
  ROS_INFO("State Machine Thread started!");

  StateMachine localFSM;	//Save state machine data locally
  //mavros_msgs::State localPX4state;
  std_msgs::UInt8 localPX4state; // DJISDK::DisplayMode::

  ros::NodeHandle n;  

  //Create service clients
  //ros::ServiceClient armClient = n.serviceClient<mavros_msgs::CommandBool>
  //("mavros/cmd/arming");
  //ros::ServiceClient SetModeClient = n.serviceClient<mavros_msgs::SetMode>
  //("mavros/set_mode");
  // drone arm service client
  ros::ServiceClient drone_arm_service = n.serviceClient<dji_sdk::DroneArmControl>
    ("dji_sdk/drone_arm_control");
  ros::ServiceClient sdk_ctrl_authority_service = n.serviceClient<dji_sdk::SDKControlAuthority>
    ("dji_sdk/sdk_control_authority");

  //mavros_msgs::CommandBool ArmMsg;
  //ArmMsg.request.value = true;
  //mavros_msgs::SetMode ModeMsg;
  //ModeMsg.request.custom_mode = "OFFBOARD";

  //ros::Time last_request = ros::Time::now();

  while(1) {
    WaitForEvent(syncEvents.Timeout,50); //Execute every 100ms

    //Check if thread should be terminated; if so, turn motors off
    /*
       if (WaitForEvent(syncEvents.Terminate,0) == 0) {
       ArmMsg.request.value = false;
       armClient.call(ArmMsg);
       break;
       }*/
    //Check if thread should be terminated; if so, turn motors off
    if (WaitForEvent(syncEvents.Terminate,0) == 0) {
      ServiceAck service_ack;
      service_ack = disArmMotors(drone_arm_service); // disarm motors
      if (service_ack.result) {
        ROS_INFO("ArmMotors command sent successfully");
      } else {
        ROS_WARN("Failed sending armMotors command");
      }
      break;
    }


    //Check events for change of state and print when changing states
    // NOTE: use if instead of switch, because of finite state machine
    if (WaitForEvent(joyEvents.buttonA,0) == 0) {
      ROS_INFO("Disarming...");
      pthread_mutex_lock(&mutexes.FSM);
      FSM.State = FSM.MODE_DISARM;
      pthread_mutex_unlock(&mutexes.FSM);
    }
    if (WaitForEvent(joyEvents.buttonB,0) == 0) {
      pthread_mutex_lock(&mutexes.FSM);
      if(FSM.State != FSM.MODE_POSITION_ROS){
        ROS_INFO("ROS Position Mode!");
      }
      FSM.State = FSM.MODE_POSITION_ROS;
      pthread_mutex_unlock(&mutexes.FSM);
      // e_ROS_PosModeSet = 1;   %Flag that tells the RefPubThread that this mode was just enabled
      // TODO: checkout RefPubThread
    }
    if (WaitForEvent(joyEvents.buttonX,0) == 0) {
      ROS_INFO("Joystick Position Mode!");
      pthread_mutex_lock(&mutexes.FSM);
      FSM.State = FSM.MODE_POSITION_JOY;
      pthread_mutex_unlock(&mutexes.FSM);
    }
    if (WaitForEvent(joyEvents.buttonY,0) == 0) {
      ROS_INFO("Joystick Attitude Mode!");
      pthread_mutex_lock(&mutexes.FSM);
      FSM.State = FSM.MODE_ATTITUDE;
      pthread_mutex_unlock(&mutexes.FSM);
    }
    if (WaitForEvent(joyEvents.buttonLeft,0) == 0) {
      ROS_INFO("Local Position control Mode!");
      pthread_mutex_lock(&mutexes.FSM);
      if(FSM.PosControlMode == FSM.POS_CONTROL_LOCAL){
        if(FSM.PosRefMode != FSM.POS_REF_WORLD){
          FSM.PosRefMode = FSM.POS_REF_WORLD;
          ROS_INFO("Joystick reference is in world frame!");
        }
        else{
          FSM.PosRefMode = FSM.POS_REF_BODY;
          ROS_INFO("Joystick reference is in body frame!");
        }
      }
      FSM.PosControlMode = FSM.POS_CONTROL_LOCAL;		    	
      pthread_mutex_unlock(&mutexes.FSM);
    }
    if (WaitForEvent(joyEvents.buttonRight,0) == 0) {
      ROS_WARN("Native Position control Mode not supported!");
    }
    /*
    if (WaitForEvent(joyEvents.buttonRight,0) == 0) {
      ROS_INFO("PX4 Position control Mode!");
      pthread_mutex_lock(&mutexes.FSM);
      if(FSM.PosControlMode == FSM.POS_CONTROL_PX4){
        if(FSM.PosRefMode != FSM.POS_REF_WORLD){
          FSM.PosRefMode = FSM.POS_REF_WORLD;
          ROS_INFO("Joystick reference is in world frame!");
        }
        else{
          FSM.PosRefMode = FSM.POS_REF_BODY;
          ROS_INFO("Joystick reference is in body frame!");
        }
      }
      FSM.PosControlMode = FSM.POS_CONTROL_PX4;
      pthread_mutex_unlock(&mutexes.FSM);
    }
    */
    if (WaitForEvent(joyEvents.buttonSelect,0) == 0) {
      ROS_INFO("Terminating Node!");
      SetEvent(syncEvents.Terminate);
    }

    //Get information about state of the system
    pthread_mutex_lock(&mutexes.FSM);
    localFSM = FSM;
    pthread_mutex_unlock(&mutexes.FSM);

    //Get PX4 state
    pthread_mutex_lock(&mutexes.PX4state);
    localPX4state = PX4state;
    pthread_mutex_unlock(&mutexes.PX4state);

    //Request to arm depending on desired state
    //Chunk of code extracted from 
    if( (localFSM.State == localFSM.MODE_POSITION_JOY) ||
        (localFSM.State == localFSM.MODE_POSITION_ROS) ||
        (localFSM.State == localFSM.MODE_ATTITUDE)) {

      // obtain control authority
      ServiceAck service_ack_authority;
      //service_ack = obtainCtrlAuthority();
      service_ack_authority = obtainCtrlAuthority(sdk_ctrl_authority_service );
      if (service_ack_authority.result) {
        ROS_INFO("Obtain SDK control Authority successfully");
      } else {
        ROS_WARN("Failed Obtain SDK control Authority");
      }
      // arm the drone
      ServiceAck service_ack_arm;
      service_ack_arm = disArmMotors(drone_arm_service); // disarm motors
      if (service_ack_arm.result) {
        ROS_INFO("disArmMotors command sent successfully");
      } else {
        ROS_WARN("Failed sending disArmMotors command");
      }

      /*
         if( localPX4state.mode != "OFFBOARD" &&
         (ros::Time::now() - last_request > ros::Duration(1.0)) ){
      // if not offboard mode, then enable offboard
      if( SetModeClient.call(ModeMsg) &&
      ModeMsg.response.mode_sent ) { // ModeMsg.response.success -> mode_sent in new mavros version
      ROS_INFO("Offboard enabled!");
      }
      last_request = ros::Time::now();
      } else { 
      // if alreay offboard, arm the vehicle
      if( !localPX4state.armed &&
      (ros::Time::now() - last_request > ros::Duration(1.0)) ){
      ArmMsg.request.value = true;
      if( armClient.call(ArmMsg) &&
      ArmMsg.response.success) {
      ROS_INFO("Vehicle armed!");
      }
      last_request = ros::Time::now();
      }
      }*/

    } else {
      ServiceAck service_ack_disarm;
      //service_ack = disArmMotors(); // disarm motors
      service_ack_disarm = disArmMotors(drone_arm_service); // disarm motors
      if (service_ack_disarm.result) {
        ROS_INFO("disArmMotors command sent successfully");
      } else {
        ROS_WARN("Failed sending disArmMotors command");
      }
      //ArmMsg.request.value = false;
      //armClient.call(ArmMsg);
    }

  }

  ROS_INFO("Exiting State Machine Thread...");

  pthread_mutex_lock(&mutexes.threadCount);
  threadCount -= 1;
  pthread_mutex_unlock(&mutexes.threadCount);
  pthread_exit(NULL);
}

ServiceAck armMotors(ros::ServiceClient service) {
  dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = 1;
  //drone_arm_service.call(droneArmControl);
  service.call(droneArmControl);
  if(!droneArmControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneArmControl.response.cmd_set, droneArmControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneArmControl.response.ack_data);
  }
  return {droneArmControl.response.result, droneArmControl.response.cmd_set,
    droneArmControl.response.cmd_id, droneArmControl.response.ack_data};
}

ServiceAck disArmMotors(ros::ServiceClient service) {
  dji_sdk::DroneArmControl droneArmControl;
  droneArmControl.request.arm = 0;
  //drone_arm_service.call(droneArmControl);
  service.call(droneArmControl);
  if(!droneArmControl.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", droneArmControl.response.cmd_set, droneArmControl.response.cmd_id);
    ROS_WARN("ack.data: %i", droneArmControl.response.ack_data);
  }
  return {droneArmControl.response.result, droneArmControl.response.cmd_set,
    droneArmControl.response.cmd_id, droneArmControl.response.ack_data};
}

ServiceAck obtainCtrlAuthority(ros::ServiceClient service) {
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 1;
  //sdk_ctrl_authority_service.call(sdkAuthority);
  service.call(sdkAuthority);
  if(!sdkAuthority.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return {sdkAuthority.response.result, sdkAuthority.response.cmd_set,
    sdkAuthority.response.cmd_id, sdkAuthority.response.ack_data};
}

ServiceAck releaseCtrlAuthority(ros::ServiceClient service) {
  dji_sdk::SDKControlAuthority sdkAuthority;
  sdkAuthority.request.control_enable = 0;
  //sdk_ctrl_authority_service.call(sdkAuthority);
  service.call(sdkAuthority);
  if(!sdkAuthority.response.result) {
    ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set, sdkAuthority.response.cmd_id);
    ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
  }
  return {sdkAuthority.response.result, sdkAuthority.response.cmd_set,
    sdkAuthority.response.cmd_id, sdkAuthority.response.ack_data};
}

