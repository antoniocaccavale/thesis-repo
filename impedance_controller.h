// ROS headers
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
    void print();
    void run();


  private:

    ros::NodeHandle nh;
    ros::Subscriber pose_sub;
    icaros_object_rcg::icr_msg_box box_ref;
    bool read;
    bool done;



};
