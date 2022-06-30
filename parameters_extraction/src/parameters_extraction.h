#include "ros/ros.h"
#include "tf/tf.h"
#include "boost/thread.hpp"
#include <iostream>
#include <rbdl/rbdl.h>
#include <rbdl/Model.h>
#include <rbdl/ModelData.h>
#include <rbdl/Dynamics.h>
#include <rbdl/Kinematics.h>
#include <rbdl/rbdl_math.h>
#include <rbdl/rbdl_mathutils.h>
#include <rbdl/Logging.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace Math;
using namespace Eigen;




class PARAM_EXTRACT{
	public:
		PARAM_EXTRACT();

		void run();
		void extract();
		void joint_states_cb(sensor_msgs::JointState js);
		void link_states_cb(gazebo_msgs::LinkStates ls);
	
		
		
	private:
		

		ros::NodeHandle nh_;
		RigidBodyDynamics::Model rbdl_model_;  /*!< Robot model from RBDL */
		std::vector<std::string> joint_names_;                  /*!< Vector with the joint names of all the joints, including the actuated and the static ones */
		sensor_msgs::JointState js_;
		gazebo_msgs::LinkStates ls_;
		geometry_msgs::Pose ee_l_;
		geometry_msgs::Pose ee_r_;
		ros::Subscriber joint_states_sub_;
		ros::Subscriber link_states_sub_;
		int dofs_=7;
		
		RigidBodyDynamics::Model arm_left_model_;
		RigidBodyDynamics::Model arm_right_model_;


};






