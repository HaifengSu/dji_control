#include "commPub.h"

void *commPubTimer(void *threadID){

  ROS_INFO("commPubTimer has started!");
  int SamplingTime = 4;	//Sampling time in milliseconds

  while(1){
    WaitForEvent(syncEvents.Timeout,SamplingTime);

    //Check if thread should be terminated
    if(WaitForEvent(syncEvents.Terminate,0) == 0){
      break;
    }

    SetEvent(syncEvents.CommPub_trigger);
  }

  ROS_INFO("Exiting commPubTimer...");

  //Shutdown here
  pthread_mutex_lock(&mutexes.threadCount);
  threadCount -= 1;
  pthread_mutex_unlock(&mutexes.threadCount);
  pthread_exit(NULL);
}

void *commPubTask(void *threadID){
  ROS_INFO("Command Publisher started!");

  //Local variables
  joyStruct localJoy; 			//Used to save global variable into local scope
  StateMachine localFSM;			//Save state machine data locally
  nav_msgs::Odometry localOdom;	//Save odometry data locally
  PVA_structure localPVA_ref;
  PosControlParam localParam;
  std_msgs::Float64 refThrust;
  geometry_msgs::PoseStamped  PoseRef;

  //Publishers
  ros::NodeHandle n; 
  //ros::Publisher PosPub = n.advertise<geometry_msgs::PoseStamped>
  //("mavros/setpoint_position/local",100);
  //ros::Publisher AttPub = n.advertise<geometry_msgs::PoseStamped>
  //("mavros/setpoint_attitude/attitude",100);
  //ros::Publisher ThrustPub = n.advertise<std_msgs::Float64>
  //("mavros/setpoint_attitude/att_throttle", 100);
  ros::Publisher djiPub = n.advertise<sensor_msgs::Joy>
    ("dji_sdk/flight_control_setpoint_generic",100);

  ros::Time t_prev = ros::Time::now();
  ros::Time t_now = ros::Time::now();
  ros::Duration dt;

  int count = 0;
  while(1){
    WaitForEvent(syncEvents.CommPub_trigger,500);

    //Check if thread should be terminated
    if(WaitForEvent(syncEvents.Terminate,0) == 0){
      break;
    }

    //Ellapsed time calculations
    t_now = ros::Time::now();
    dt = t_now - t_prev;
    t_prev = t_now;

    //Get latest joystick values
    pthread_mutex_lock(&mutexes.joy);
    localJoy = joy;
    pthread_mutex_unlock(&mutexes.joy);

    //Get information about state of the system
    pthread_mutex_lock(&mutexes.FSM);
    localFSM = FSM;
    pthread_mutex_unlock(&mutexes.FSM);

    //Get odometry info
    pthread_mutex_lock(&mutexes.odom);
    localOdom = odom;
    pthread_mutex_unlock(&mutexes.odom);

    //Get reference PVA
    if(localFSM.State == localFSM.MODE_POSITION_ROS){
      // ROS position mode
      // set PVA_ros to localPVA_ref
      pthread_mutex_lock(&mutexes.PVA_ros);
      localPVA_ref.Pos.pose.position = PVA_Ros.Pos;
      localPVA_ref.Pos.pose.orientation = 
        rpy2quat(SetVector3(0, 0, PVA_Ros.yaw));
      localPVA_ref.Vel.twist.linear = PVA_Ros.Vel;
      localPVA_ref.Acc.accel.linear = PVA_Ros.Acc;
      pthread_mutex_unlock(&mutexes.PVA_ros);
    } else{
      // other modes, set PVAref as localPVA_ref
      pthread_mutex_lock(&mutexes.PVAref);
      localPVA_ref = PVA_ref;
      pthread_mutex_unlock(&mutexes.PVAref);
    }

    //Get position controller parameters
    pthread_mutex_lock(&mutexes.PID_Param);
    localParam = ControlParam;
    pthread_mutex_unlock(&mutexes.PID_Param);

    //Use appropriate settings depending on mode
    if( (localFSM.State == localFSM.MODE_POSITION_JOY) ||
        (localFSM.State == localFSM.MODE_POSITION_ROS)) {
      // joystick position or ros position mode

      if (localFSM.PosControlMode == localFSM.POS_CONTROL_LOCAL){
        // local position control (local frame?) 
        pthread_mutex_lock(&mutexes.PID_Pos);
        // call controller, given local reference position, odometry, etc.
        PosController(localOdom, localPVA_ref, localParam, dt.toSec(),
            PosPID, PoseRef, refThrust);
        pthread_mutex_unlock(&mutexes.PID_Pos);
      } else { // no control
        refThrust.data = 0;
        PoseRef.pose.orientation = setQuat(0, 0, 0, 1);
        geometry_msgs::Point position;
        position.x = 0; position.y = 0; position.z = 0;
        PoseRef.pose.position = position;
      }
      /*else if (localFSM.PosControlMode == localFSM.POS_CONTROL_PX4){
        // PX4 position control (using PX4 controller)
        refThrust.data = 0;
        PoseRef.pose.orientation = setQuat(0, 0, 0, 1);
        PoseRef.pose = localPVA_ref.Pos.pose;
      } */
    } else if (localFSM.State == localFSM.MODE_ATTITUDE) {
      // (joystick) attitude mode

      refThrust = localPVA_ref.thrustRef;
      PoseRef.pose = localPVA_ref.Pos.pose;
    } else {
      // other modes

      refThrust.data = 0;
      PoseRef.pose = localOdom.pose.pose;
    }

    //Set header for reference
    PoseRef.header.seq = count;
    PoseRef.header.stamp = t_now;
    PoseRef.header.frame_id = "fcu";

    //Publish references
    /*
    if( (localFSM.PosControlMode == localFSM.POS_CONTROL_LOCAL) ||
        (localFSM.State == localFSM.MODE_ATTITUDE)) {
      AttPub.publish(PoseRef);
      ThrustPub.publish(refThrust);
    }
    // POS_CONTROL_PX4 mode is useless without GPS, same as dji 
    else if (localFSM.PosControlMode == localFSM.POS_CONTROL_PX4){
      PosPub.publish(PoseRef);
    }
    */

    //Publish atitude control reference
    if( (localFSM.PosControlMode == localFSM.POS_CONTROL_LOCAL) ||
        (localFSM.State == localFSM.MODE_ATTITUDE)) {
      //AttPub.publish(PoseRef);
      //ThrustPub.publish(refThrust);

      // publish dji generic control commands (roll,pitch,thrust,yaw)
      geometry_msgs::Vector3 rpy = quat2rpy(PoseRef.pose.orientation);
      float thrust = refThrust.data;

      sensor_msgs::Joy controlAttitudeThrust;
      uint8_t flag = (DJISDK::VERTICAL_THRUST |
          DJISDK::HORIZONTAL_ANGLE    |
          DJISDK::YAW_ANGLE           |
          DJISDK::HORIZONTAL_BODY     |
          DJISDK::STABLE_ENABLE);
      controlAttitudeThrust.axes.push_back(rpy.x);
      controlAttitudeThrust.axes.push_back(rpy.y);
      controlAttitudeThrust.axes.push_back(thrust);
      controlAttitudeThrust.axes.push_back(rpy.z);
      controlAttitudeThrust.axes.push_back(flag);

      djiPub.publish(controlAttitudeThrust);
    }

    count += 1;
  }

  ROS_INFO("Exiting Command Publisher Thread...");

  pthread_mutex_lock(&mutexes.threadCount);
  threadCount -= 1;
  pthread_mutex_unlock(&mutexes.threadCount);
  pthread_exit(NULL);

}
