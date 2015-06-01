/** \file robotnik_tilt_laser.cpp
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2014
 *
 * \brief robotnik_tilt_laser ros node
 * Component to manage the robotnik tilting laser = hokuyo laser + dynamixel pro motor
 * (C) 2014 Robotnik Automation, SLL
 * uses ROBOTS Dynamixel PRO SDK
*/
#include <string.h>
#include <vector>
#include <stdint.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <cstdlib>

//#include "ros/time.h"
#include "self_test/self_test.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "diagnostic_updater/update_functions.h"
#include "diagnostic_updater/DiagnosticStatusWrapper.h"
#include "diagnostic_updater/publisher.h"

#include "dynamixel.h"
#include "bulkread.h"

#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <robotnik_msgs/set_float_value.h>

// Messages
#include "sensor_msgs/PointCloud.h"

// Services
#include "laser_assembler/AssembleScans.h"

#define	 DXL_MIN_COMMAND_REC_FREQ	1.0
#define	 DXL_MAX_COMMAND_REC_FREQ	20.0

// Pps to angle conversions
#define  PPS2DEG 0.001185185 	// 180.0/151875.0
#define  PPS2RAD 0.000020685 	// PI/151875.0 

//#define  TILT_ANGLE_UP_DEG  11.85   // Tilting angle +      (10000)  home = -31000
//#define  TILT_ANGLE_DOWN_DEG -100.74  // Titlting angle -   (-85000)
#define  TILT_ANGLE_HOME_PPS -31000
#define  TILT_ANGLE_UP_PPS   10000   // (TILT_ANGLE_UP_DEG / PPS2DEG)
#define  TILT_ANGLE_DOWN_PPS -85000  // (TILT_ANGLE_DOWN_DEG / PPS2DEG)


// Control table addresses
#define OPERATING_MODE     11
#define MAX_VELOCITY       32
#define TORQUE_ENABLE      562
#define GOAL_POSITION      596
#define GOAL_TORQUE	   	   604
#define GOAL_VELOCITY	   600

#define PRESENT_CURRENT    621
#define PRESENT_VELOCITY   615
#define PRESENT_POSITION   611

// other settings
#define DEFAULT_BAUDNUM     1 // 57600 bps to match Dynamixel PRO's
#define CONTROL_PERIOD      (10) // arbitrary value

// Stop conditions can be adjusted to requirements
#define  STOP_RANGE_DEG  1.0    // Stop angle (stop before profile programmed limit
#define  STOP_RANGE_PPS  843	// (STOP_RANGE_DEG / PPS2DEG)

#define STATE_STOP       1
#define STATE_START      2
#define STATE_HOME       3
#define STATE_POSITION   4
#define STATE_DEFINE_VELOCITY   5

//using namespace std;
using namespace DXL_PRO;

class laser_tilt_node
{

public:
	self_test::TestRunner self_test_;
	ros::NodeHandle node_handle_;
	ros::NodeHandle private_node_handle_;
	int error_count_;
	int slow_count_;
	double desired_freq_;
	diagnostic_updater::Updater diagnostic_;			// General status diagnostic updater
	diagnostic_updater::FrequencyStatus freq_diag_;		         // Component frequency diagnostics
        diagnostic_updater::HeaderlessTopicDiagnostic *subs_command_freq; // Topic reception frequency diagnostics
	ros::Time last_command_time;					// Last moment when the component received a command
        diagnostic_updater::FunctionDiagnosticTask command_freq_;

	// Current axis position in pulses
	long position_pps_;
	
	//! Parameters
	//! Frame id strings
	std::string base_laser_frame_id;
	std::string tilt_laser_frame_id;
	//! Port for USB DXL converter
	std::string port_;
	//! Displacement from base_laser_frame_id to tilt_laser_frame_id
	double x_offset_, y_offset_, z_offset_;
	
	//! Broadcast tf
	tf::TransformBroadcaster tf_broadcaster;

	//! Motor driver component 
	Dynamixel* driver;
	
	//! Node running
	bool running;	

	//! Error counters and flags
	std::string was_slow_;
	std::string error_status_;

	//! Internal state
	int iState_;

	//! Internal state in start state
	int iStartState_;
	
	//! Target position when called from external service
	float target_position_;
	
	//! Velocity when performing a 
	double target_velocity_;
	
	//! Ros service stop 
	ros::ServiceServer srv_stop_;
	//! Ros service start 
	ros::ServiceServer srv_start_;
	//! Ros service home
	ros::ServiceServer srv_home_;
	//! Ros service set target position
	ros::ServiceServer srv_setpos_;
	//! Ros service set velocity
	ros::ServiceServer srv_setvel_;

    //! Publisher of the joint state
    ros::Publisher joint_state_pub_;
	//! Joint state message
    sensor_msgs::JointState robot_joint_state_;
    
    bool publish_tf_;
    bool publish_joint_;
    std::string joint_name_;

	// Cloud publish
	ros::Publisher cloud_pub_;
	ros::ServiceClient assemble_scans_client_;
		
/*!	\fn laser_tilt_node::laser_tilt_node()
 * 	\brief Public constructor
*/
laser_tilt_node(ros::NodeHandle h) : self_test_(), diagnostic_(),
  node_handle_(h), private_node_handle_("~"), 
  error_count_(0),
  slow_count_(0),
  desired_freq_(50),
  freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.05)   ),
  command_freq_("Command frequency check", boost::bind(&laser_tilt_node::check_command_subscriber, this, _1))
{
    running = false;
    ros::NodeHandle laser_tilt_node_handle(node_handle_, "laser_tilt_node");
    private_node_handle_.param<std::string>("port", port_, "/dev/ttyUSB0");
    private_node_handle_.param<std::string>("base_laser_frame_id", base_laser_frame_id, "laser");
    private_node_handle_.param<std::string>("tilt_laser_frame_id", tilt_laser_frame_id, "tilt_laser");
    private_node_handle_.param<double>("x_offset", x_offset_, 0.0);
    private_node_handle_.param<double>("y_offset", y_offset_, 0.0);
    private_node_handle_.param<double>("z_offset", z_offset_, 0.0);
    private_node_handle_.param<double>("velocity", target_velocity_, 1000.0);   // max 10300

	//! Publish transform
    private_node_handle_.param("publish_tf", publish_tf_, true);
	//! Publish transform
    private_node_handle_.param("publish_joint", publish_joint_, false);
    private_node_handle_.param<std::string>("joint_name", joint_name_, "hokuyo_frontal_laser_rotation_joint");
    if (publish_tf_) ROS_INFO("laser_tilt_node - configured to publish tf");
    if (publish_joint_) ROS_INFO("laser_tilt_node - configured to publish joint_states");

    // Self test
    self_test_.add("Connect Test", this, &laser_tilt_node::ConnectTest);  

    // Component frequency diagnostics
    diagnostic_.setHardwareID("laser_tilt_node");
    diagnostic_.add("Motor Controller", this, &laser_tilt_node::controller_diagnostic);
    diagnostic_.add( freq_diag_ );
    diagnostic_.add( command_freq_ );
    
    // Topics freq control 
    // For /laser_tilt_node/command
    double min_freq = DXL_MIN_COMMAND_REC_FREQ; // If you update these values, the
    double max_freq = DXL_MAX_COMMAND_REC_FREQ; // HeaderlessTopicDiagnostic will use the new values.
    ROS_INFO("Desired freq %5.2f", desired_freq_);

    // Create new motor driver
    // ToDo fix 	
    // driver = new Dynamixel(port_.c_str());
    driver = new Dynamixel("/dev/ttyUSB0");

    // Open device
    if( driver->Connect() == 0 ) {
        ROS_ERROR("Failed to open USB2Dynamixel!" );
        exit(-1);
		}
    else
        ROS_INFO("Succeed to open USB2Dynamixel!");

    if(driver->SetBaudrate(DEFAULT_BAUDNUM) == true) {
    	ROS_INFO( "Succeed to change the baudrate!" );
		}
    else {
        ROS_ERROR( "Failed to change the baudrate!\n" );
        exit(-1);
		}

    //! Service to stop component
	srv_stop_ = private_node_handle_.advertiseService("stop", &laser_tilt_node::srvCallback_Stop, this);
    //! Service to start component
	srv_start_ = private_node_handle_.advertiseService("start", &laser_tilt_node::srvCallback_Start, this);
    //! Service to home component
	srv_home_ = private_node_handle_.advertiseService("home", &laser_tilt_node::srvCallback_Home, this);
    //! Service to set target pos
	srv_setpos_ = private_node_handle_.advertiseService("set_position", &laser_tilt_node::srvCallback_SetPosition, this);					
	//! Service to set target vel
	srv_setvel_= private_node_handle_.advertiseService("set_velocity", &laser_tilt_node::srvCallback_SetVelocity, this);					
	
	//! Publish joint states for wheel motion visualization
	joint_state_pub_ = private_node_handle_.advertise<sensor_msgs::JointState>("/joint_states", 10);

	//! Initialize robot_joint JointMessage structure
	robot_joint_state_.name.resize(1);
	robot_joint_state_.position.resize(1);
	robot_joint_state_.velocity.resize(1);
	robot_joint_state_.effort.resize(1);
	robot_joint_state_.name[0] = joint_name_;
	robot_joint_state_.position[0] = 0.0;
	robot_joint_state_.velocity[0] = 0.0;
	robot_joint_state_.effort[0] = 0.0;

		
    // Create a publisher for the clouds that we assemble
    cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud> ("assembled_cloud", 1);

    // Create the service client for calling the assembler
    assemble_scans_client_ = node_handle_.serviceClient<laser_assembler::AssembleScans>("assemble_scans");
}


/*!	\fn laser_tilt_node::~laser_tilt_node()
 * 	\brief Public destructor
*/
~laser_tilt_node(){

	stop();
   
    if (driver!=NULL) {   
		// DIsconnect USB2Dynamixel before ending program
		ROS_INFO("laser_tilt_node::Disconnecting USB2Dynamixel converter");
		driver->Disconnect();
		delete driver;
		}
}

/*!	\fn int laser_tilt_node::start()
 * 	\brief Start Controller
*/
int start(){

	stop();
    
    int max_velocity;
    int error;
    driver->ReadWord(1, MAX_VELOCITY, (int*) &max_velocity, &error);
    usleep(CONTROL_PERIOD*200);
    ROS_INFO("laser_tilt_node::read_and_publish - read max_velocity = %d", max_velocity);
    if ((target_velocity_ > max_velocity) || (target_velocity_ < -max_velocity)) ROS_ERROR("Set velocity is out of range");
    else {
       int GoalVel = (int) target_velocity_;
       driver->WriteDWord(1, GOAL_VELOCITY, GoalVel, &error);
       usleep(CONTROL_PERIOD*500);
       if (!error) ROS_INFO("laser_tilt_node::start - Setting speed to %d", GoalVel);
       }

    // Motor configured in joint mode
    // turn Torque Enable (562) on
    driver->WriteByte(1, TORQUE_ENABLE, 1, 0);
    usleep(CONTROL_PERIOD*100);
    
    // Internal state initialization
	iState_ = 0;
	iStartState_ = 0;	
	target_position_ = 0.0;

	freq_diag_.clear();
	running = true;
	return 0;
}

/*!	\fn laser_tilt_node::stop()
 * 	\brief Stop Controller
*/
int stop(){

    ROS_INFO("Stopping driver");
    if(running)
    {    	
		sleep(1);
		running = false;
    }
    return 0;
}

/*!	\fn laser_tilt_node::ConnectTest()
 * 	\brief Test to connect to Motors
*/
void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
{
   // Connection test or ping test
   int error;
   int ret = -1;
   if (driver!=NULL) 
		ret = driver->Ping(1, &error);
   if (error) {
	   ROS_ERROR("Connection Failed.");
	   process_error_code( error );
	   }
   else status.summary(0, "Connected successfully.");
}


/*!	\fn laser_tilt_node::controller_diagnostic
 * 	\brief Checks the status of the driver Diagnostics
*/
void controller_diagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	// add and addf are used to append key-value pairs.
	// stat.addf("Controller StatusWord (HEX)", "%x", sw ); // Internal controller status
	stat.add("Encoder position", (int) position_pps_ );
}

/*!	\fn laser_tilt_node::check_command_subscriber
 * 	\brief Checks that the robot is receiving at a correct frequency the command messages. Diagnostics
*/
void check_command_subscriber(diagnostic_updater::DiagnosticStatusWrapper &stat)
{
	ros::Time current_time = ros::Time::now();
/*
	double diff = (current_time - last_command_time).toSec();
	if(diff > 1.0){
		stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Topic is not receiving commands");
		//ROS_INFO("check_command_subscriber: %lf seconds without commands", diff);
	 	// if (driver) driver->SetDesiredSpeed(0.0, 0.0);
	}else{
		stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Topic receiving commands");
	}
*/
}

/*!	\fn laser_tilt_node::process_error_code
 * 	\brief Creates the appropriate error msgs in case of error
*/
void process_error_code(int ErrorCode)
{
	if(ErrorCode & ERRBIT_VOLTAGE) ROS_ERROR("Input voltage error!");
    if(ErrorCode & ERRBIT_ANGLE) ROS_ERROR("Angle limit error!");
    if(ErrorCode & ERRBIT_OVERHEAT) ROS_ERROR("Overheat error!");
    if(ErrorCode & ERRBIT_RANGE) ROS_ERROR("Out of range error!");
    if(ErrorCode & ERRBIT_CHECKSUM) ROS_ERROR("Checksum error!");
    if(ErrorCode & ERRBIT_OVERLOAD) ROS_ERROR("Overload error!");
    if(ErrorCode & ERRBIT_INSTRUCTION) ROS_ERROR("Instruction code error!");	
	// stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Controller initialing");
}

////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS


bool srvCallback_Start(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	iState_ = STATE_START;
        ROS_INFO("laser_tilt_node:: START");
	return true;
}

bool srvCallback_Stop(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	iState_ = STATE_STOP;
        ROS_INFO("laser_tilt_node:: STOP");
	return true;
}

bool srvCallback_Home(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
	iState_ = STATE_HOME;
        ROS_INFO("laser_tilt_node:: HOME");
	return true;
}

bool srvCallback_SetPosition(robotnik_msgs::set_float_value::Request  &req,
            robotnik_msgs::set_float_value::Response &res)
{
  target_position_ = req.value;
  res.ret = true;
  iState_ = STATE_POSITION;
  ROS_INFO("set_position : %5.2f", target_position_);
  return true;
}

bool srvCallback_SetVelocity(robotnik_msgs::set_float_value::Request  &req,
            robotnik_msgs::set_float_value::Response &res)
{
  target_velocity_ = req.value;
  res.ret = true;
  iState_ = STATE_DEFINE_VELOCITY;
  ROS_INFO("set_velocity : %5.2f", target_velocity_);
  return true;
}

int read_and_publish()
{
	static double prevtime = 0;
	int error = 0; 
	int ret = 0;
	static int pos_tmp = 0;
	double starttime = ros::Time::now().toSec();
	static ros::Time last_time = ros::Time::now();
	double pitch_rad  = 0.0;

	if (prevtime && prevtime - starttime > 0.05) {
		ROS_WARN("Full laser_tilt_node loop took %f ms. Nominal is 10ms.", 1000 * (prevtime - starttime));
		was_slow_ = "Full laser_tilt_node loop was slow.";
		slow_count_++;
	    }

	// For Debug
	// ROS_INFO("State:%d pos_tmp:%d", iStartState_, pos_tmp); 
    switch (iState_) {
		case 0:  // do nothing
				break;

		case STATE_STOP:
				// To stop Dynamixel PRO inmediately simply enter a velocity value of 0 (enter the command wrd 1 600 0) 
		case STATE_HOME: 
				{
				// set initial Goal Position to 0
				int GoalPos = TILT_ANGLE_HOME_PPS;
				int error;
				driver->WriteDWord(1, GOAL_POSITION, GoalPos, &error);
				usleep(CONTROL_PERIOD*500);
				if (!error) ROS_INFO("robotnik_tilt_laser::read_and_publish - Performing Home/Stop");
				else ROS_ERROR("Could not move");
				iState_ = 0;  // In both cases, after homing do rest
                                iStartState_ =0; 
			    }
				break;

		case STATE_DEFINE_VELOCITY:
				{
				// Read max velocity programmed
				int max_velocity; 	
				int error;
				driver->ReadWord(1, MAX_VELOCITY, (int*) &max_velocity, &error);							
				usleep(CONTROL_PERIOD*200);
				ROS_INFO("robotnik_tilt_laser::read_and_publish - read max_velocity = %d", max_velocity);
				
				if ((target_velocity_ > max_velocity) || (target_velocity_ < -max_velocity)) ROS_ERROR("Set velocity is out of range");
				else {
					driver->WriteByte(1, TORQUE_ENABLE, 0, &error);
					if (error) ROS_ERROR("Error disabling torque");
     					usleep(CONTROL_PERIOD*100);

					int GoalVel = target_velocity_;				
					driver->WriteDWord(1, GOAL_VELOCITY, GoalVel, &error);
					usleep(CONTROL_PERIOD*500);
					if (!error) ROS_INFO("robotnik_tilt_laser::read_and_publish - Setting speed to %d", GoalVel);			  
					else ROS_ERROR("Could not set speed");					
					driver->WriteByte(1, TORQUE_ENABLE, 1, 0);
					usleep(CONTROL_PERIOD*100);
					if (error) ROS_ERROR("Error enabling torque");

				    }
				
				iState_ = 0;
                                iStartState_ = 0;  
				}
				break;
				
		case STATE_POSITION: 
				{
				// set target position received via service
				// ToDo, may chekc absolute limits 
				int GoalPos = (int) target_position_;
				int error;
                ROS_INFO("GoalPos = %d", GoalPos);
				//driver->WriteDWord(1, GOAL_POSITION, GoalPos, &error);
				driver->WriteDWord(1, GOAL_POSITION, (int32_t) GoalPos, &error);
				usleep(CONTROL_PERIOD*500);
				if (!error) ROS_INFO("robotnik_tilt_laser::read_and_publish - Moving To Target Position");
				else ROS_ERROR("Could not move");
				iState_ = 0; 
                                iStartState_ = 0;                                
			    }
				break;
				
		case STATE_START :  
				{
				// Component state machine   
				switch (iStartState_) {
					case 0: // Program tilt up				
                            //ROS_INFO("TILT UP");
							driver->WriteDWord(1, GOAL_POSITION, TILT_ANGLE_UP_PPS, &error);
							if (error) {
								ROS_ERROR("Error writing goal position");
								process_error_code( error );
								}
							usleep(CONTROL_PERIOD*200);
							ret = driver->ReadDWord(1, PRESENT_POSITION, (long*) &position_pps_, &error);
							if (error) {
								ROS_ERROR("Error reading position");
								process_error_code( error );
								}
							else pos_tmp = (int) position_pps_;
							iStartState_=1;
							break;
					case 1: // Wait end of course (don't need to wait to full stop to command next pos)
							//ROS_INFO("WAITING TILT_UP");
                            ret = driver->ReadDWord(1, PRESENT_POSITION, (long*) &position_pps_, &error);                   
							if (error) {
								ROS_ERROR("Error reading position");
								process_error_code( error );
								}
							else pos_tmp = (int) position_pps_;
							
							if (pos_tmp>(TILT_ANGLE_UP_PPS - STOP_RANGE_PPS)) {
								iStartState_ = 2;
								/*
								// Populate our service request based on our timer callback times
								laser_assembler::AssembleScans srv;
								srv.request.begin = last_time;
								last_time = ros::Time::now();
								srv.request.end   = ros::Time::now();
								// Make the service call
								if (assemble_scans_client_.call(srv)) {
								  ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
								  cloud_pub_.publish(srv.response.cloud);
								  }
								else {
								  ROS_ERROR("Error making service call\n") ;
								  }
								*/
								}
							break;
														
					case 2: // Program tilt down                                                        
							//ROS_INFO("TILT DOWN");
							driver->WriteDWord(1, GOAL_POSITION, TILT_ANGLE_DOWN_PPS, &error);
							if (error) {
								ROS_ERROR("Error writing goal position");
								process_error_code( error );
								}
							usleep(CONTROL_PERIOD*200);
							ret = driver->ReadDWord(1, PRESENT_POSITION, (long*) &position_pps_, &error);
							if (error) {
								ROS_ERROR("Error reading position");
								process_error_code( error );
								}
							else pos_tmp = (int) position_pps_;                        
							iStartState_=3;
							break;
					case 3: // Wait end of course (don't need to wait to full stop to command next pos)
                                                        //ROS_INFO("WAITING TILT DOWN");
							ret = driver->ReadDWord(1, PRESENT_POSITION, (long*) &position_pps_, &error);
							if (error) {
								ROS_ERROR("Error reading position");      
								process_error_code( error );
								}
							pos_tmp = (int) position_pps_;
							if (pos_tmp < (TILT_ANGLE_DOWN_PPS + STOP_RANGE_PPS)) {
								iStartState_=0;				
								// Populate our service request based on our timer callback times
								laser_assembler::AssembleScans srv;
								srv.request.begin = last_time;
								last_time = ros::Time::now();
								srv.request.end   = ros::Time::now();
								// Make the service call
								if (assemble_scans_client_.call(srv)) {
								  ROS_INFO("Published Cloud with %u points", (uint32_t)(srv.response.cloud.points.size())) ;
								  cloud_pub_.publish(srv.response.cloud);
								  }
								else {
								  ROS_ERROR("Error making service call\n") ;
								  }								
								}	
															
							break;
					}

					// Compute angle for tf publishing
					pitch_rad = (pos_tmp - TILT_ANGLE_HOME_PPS) * PPS2RAD; 

					
				}
				break;
		default:
				break;
		}  // fSwitch
	
	if (publish_tf_) {
		ros::Time current_time = ros::Time::now();		
		// first we'll publish the transforms over tf
		geometry_msgs::TransformStamped laser_tilt_tf;
		laser_tilt_tf.header.stamp = current_time;
		laser_tilt_tf.header.frame_id = base_laser_frame_id;
		laser_tilt_tf.child_frame_id = tilt_laser_frame_id;
		laser_tilt_tf.transform.translation.x = x_offset_;   	// Comming from parameters
		laser_tilt_tf.transform.translation.y = y_offset_;
		laser_tilt_tf.transform.translation.z = z_offset_;
		laser_tilt_tf.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, pitch_rad, 0.0);
		tf_broadcaster.sendTransform(laser_tilt_tf);
	}
	if (publish_joint_) {
		robot_joint_state_.header.stamp = ros::Time::now();
		robot_joint_state_.position[0] = pitch_rad;
		joint_state_pub_.publish( robot_joint_state_ );
	}
								
	double endtime = ros::Time::now().toSec();
	if (endtime - starttime > 0.05) {
		ROS_WARN("Publishing took %f ms. Nominal is 10 ms.", 1000 * (endtime - starttime));
		was_slow_ = "Full tilt_laser_node loop was slow.";
		slow_count_++;
		}
	
	prevtime = starttime;
	starttime = ros::Time::now().toSec();
	freq_diag_.tick();
	return(0);
}


bool spin()
{
    ros::Rate r(desired_freq_);  // 50.0 

    while (!ros::isShuttingDown()) // Using ros::isShuttingDown to avoid restarting the node during a shutdown.
    {
      if (start() == 0)
      {
	  while(ros::ok() && node_handle_.ok()) {
          if(read_and_publish() < 0)
            break;
          self_test_.checkTest();
          diagnostic_.update();
          ros::spinOnce();
	      r.sleep();
          }
	   ROS_INFO("laser_tilt_node::spin - END OF ros::ok() !!!");
       } else {
       // No need for diagnostic here since a broadcast occurs in start
       // when there is an error.
       usleep(1000000);
       self_test_.checkTest();
       ros::spinOnce();
      }
   }

   return true;
}

	
}; // class laser_tilt_node

// MAIN
int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_tilt_node");
	
	ros::NodeHandle n;		
  	laser_tilt_node ltn(n);

  	ltn.spin();

	return (0);
}
// EOF
