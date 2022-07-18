#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>

// Std C++ headers
#include <string>
#include <vector>
#include <map>
#include <string.h>

#include <icaros_object_rcg/icr_msg_box.h>
using namespace std;

class ROS_SUB{
  
  public:

    ROS_SUB();
    void pose_cb(icaros_object_rcg::icr_msg_box box_pose);
    void control();
    //void arm_right_motion();
    //void arm_left_motion();
    void both_arms_motion();
    void run();


  private:

    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    icaros_object_rcg::icr_msg_box box_ref;
    //bool read_right;
    //bool done_right;
    //bool done_left;
    //bool read_left;
    bool done;
    bool read;


};