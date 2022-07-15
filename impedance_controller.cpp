#include "impedance_controller.h"



ROS_SUB::ROS_SUB(){

  pose_sub = nh.subscribe("/icaros_object_recognition/box_pose_points", 1, &ROS_SUB::pose_cb, this);
  read=false;
  done=false;
}

void ROS_SUB::pose_cb(icaros_object_rcg::icr_msg_box box_pose){

  box_ref=box_pose;

}

void ROS_SUB::print(){

  ros::Rate r(100);

  while(ros::ok() && !done){

    cout<<"Des Pose: "<<endl;
    cout<<"X:"<<box_ref.Right_box_pose.position.x<<"   Y:"<<box_ref.Right_box_pose.position.y<<"   Z:"<<box_ref.Right_box_pose.position.x<<endl<<endl;
    
    if((box_ref.Right_box_pose.position.x!=0) && (box_ref.Right_box_pose.position.y!=0) &&(box_ref.Right_box_pose.position.z!=0)){
      read=true;
    }

    if(read){

      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.frame_id = "base_footprint";
      std::string arm_name = "right";
      goal_pose.pose=box_ref.Right_box_pose;
      std::vector<std::string> torso_arm_joint_names;
      //select group of joints
      std::string moveit_group = "arm_" + arm_name + "_torso";
      moveit::planning_interface::MoveGroupInterface group_arm_torso(moveit_group);
      //choose your preferred planner
      group_arm_torso.setPlannerId("SBLkConfigDefault");
      group_arm_torso.setPoseReferenceFrame("base_footprint");
      group_arm_torso.setPoseTarget(goal_pose);

      ROS_INFO_STREAM("Planning to move " <<
                      group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                      group_arm_torso.getPlanningFrame());

      group_arm_torso.setStartStateToCurrentState();
      group_arm_torso.setMaxVelocityScalingFactor(1.0);


      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      //set maximum time to find a plan
      group_arm_torso.setPlanningTime(10.0);
      bool success = bool(group_arm_torso.plan(my_plan));

      if ( !success )
        throw std::runtime_error("No plan found");

      ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

      // Execute the plan
      ros::Time start = ros::Time::now();


      moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
      if (!bool(e))
        throw std::runtime_error("Error executing plan");

      ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());
      done=true;
      

      }
      
      r.sleep();

}

cout<<"\n";
cout << "Printing finishes" << endl;

}


void ROS_SUB::run(){
      boost::thread print_thr( &ROS_SUB::print, this);
      ros::spin();      
}


int main(int argc, char** argv){   
   ros::init(argc,argv, "impedance_controller_node");
   ROS_SUB rs;
   rs.run();
   return 0;
}