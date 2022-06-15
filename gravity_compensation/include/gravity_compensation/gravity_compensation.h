#include "ros/ros.h"
#include "tf/tf.h"
#include "boost/thread.hpp"
#include <iostream>
#include <rbdl/rbdl.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <pluginlib/class_list_macros.h>
#include <rbdl/Dynamics.h>
#include <rbdl/Kinematics.h>
#include <controller_interface/controller_base.h>
#include <controller_interface/internal/robothw_interfaces.h>
#include <hardware_interface/internal/demangle_symbol.h>
#include <hardware_interface/hardware_interface.h>
#include <cmath>
#include <controller_interface/controller.h>
#include <eigen3/Eigen/Dense>
#include <XmlRpc.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

using namespace std;
using namespace Eigen;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Addons;


namespace force_control
{

	enum JointType
		{
		STATIC, /*!< Joint not actuated in the controller */
		ACTUATED, /*!< Joint actuaded in the controller. Resource of the controller */
		ACTUATED_NO_CONTROL /*!< Joint commanded constantly with zero effort. Resource of the controller */
		};


		
		struct GravityCompensationParameters
		{
		double static_friction = 0.0;
		double viscous_friction = 0.0;
		double velocity_tolerance = 0.0;
		};

		
		struct ActuatorParameters
		{
		double motor_torque_constant = 0.0;
		double reduction_ratio = 0.0;
		};

		
		struct ActuatedJoint
		{
		hardware_interface::JointHandle joint_handle;
		ActuatorParameters actuator_parameters;
		GravityCompensationParameters friction_parameters;
		};
	

class GravityCompensation : public controller_interface::ControllerBase{
	public:
		bool initRequest(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh, ClaimedResources& claimed_resources);	
		void update(const ros::Time& /*time*/, const ros::Duration& /*period*/);
		void starting(const ros::Time& time);
  		void stopping(const ros::Time& time);

			
	private:
		
		bool init(hardware_interface::EffortJointInterface* effort_iface, hardware_interface::JointStateInterface* joint_state_iface, ros::NodeHandle& /*root_nh*/, ros::NodeHandle& control_nh);
		RigidBodyDynamics::Model rbdl_model_;  /*!< Robot model from RBDL */
		std::vector<std::string> joint_names_;                  /*!< Vector with the joint names of all the joints, including the actuated and the static ones */
		std::map<std::string, JointType> joint_types_;          /*!< Map to define which joint are actuated and which one are static */
		std::map<std::string, ActuatedJoint> actuated_joints_;  /*!< Map with the actuated joints and his parameters + hardware interface to read and write commands*/
		std::map<std::string, hardware_interface::JointStateHandle> static_joints_;   /*!< Map with the static joints and his hardware interface to read the current position */
		Eigen::VectorXd q_zero_;    /*!< Zero vector with joint_names size */
  		Eigen::VectorXd tau_cmd_;   /*!< Vector with the necessary torque to maintain gravity */
  		Eigen::VectorXd q_act_;     /*!< Vector with the current position of the joint states */
	//	ddynamic_reconfigure::DDynamicReconfigurePtr ddr_; 
};

}

PLUGINLIB_EXPORT_CLASS(force_control::GravityCompensation, controller_interface::ControllerBase)




