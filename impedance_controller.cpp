
#include "impedance_controller.h"



ROS_SUB::ROS_SUB(){

  pose_sub = nh.subscribe("/icaros_object_recognition/box_pose_points", 1, &ROS_SUB::pose_cb, this);
  //read_right=false;
  //done_right=false;
  //read_left=false;
  //done_left=false;
  read=false;
  done=false;
}

void ROS_SUB::pose_cb(icaros_object_rcg::icr_msg_box box_pose){

  box_ref=box_pose;

}

/*
void ROS_SUB::arm_right_motion(){

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
      group_arm_torso.setPlanningTime(20.0);
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
      
}

void ROS_SUB::arm_left_motion(){

      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.frame_id = "base_footprint";
      std::string arm_name = "left";
      goal_pose.pose=box_ref.Left_box_pose;
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
      group_arm_torso.setPlanningTime(20.0);
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
      
}
*/

void ROS_SUB::both_arms_motion(){

  geometry_msgs::PoseStamped Left_Arm_pose;
  geometry_msgs::PoseStamped Right_Arm_pose;
  Left_Arm_pose.pose.position.x = 0.4;  /*  Bigger Than */
  Left_Arm_pose.pose.position.y = box_ref.Left_box_pose.position.y + 0.1;   //0.2
  Left_Arm_pose.pose.position.z = box_ref.Left_box_pose.position.z + 0.1;
  Left_Arm_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -0.78);


  Right_Arm_pose.pose.position.x = 0.4 ;  /* Bigger Than 0.5*/
  Right_Arm_pose.pose.position.y = (box_ref.Right_box_pose.position.y + box_ref.Right_box_pose.position.y)/2;
  Right_Arm_pose.pose.position.z = box_ref.Right_box_pose.position.z - 0.1 ;  /*0.7 to 1.1*/
  Right_Arm_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);    /*0, 0.1, 0.2, 0.3 upper body*/

  std::string moveit_group_selection; 
  moveit_group_selection = "both_arms_torso";
  Left_Arm_pose.header.frame_id  = "base_footprint";
  Right_Arm_pose.header.frame_id = "base_footprint";
  

  /* ------ Set motion planning type ------- */
  moveit::planning_interface::MoveGroupInterface group_arm_torso(moveit_group_selection);
  group_arm_torso.setPoseTarget(Left_Arm_pose,"arm_left_tool_link");
  group_arm_torso.setPoseTarget(Right_Arm_pose,"arm_right_tool_link"); 
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);
  group_arm_torso.setPoseReferenceFrame("base_footprint");

  /* ------ Set the Maximum time to find the plane ------- */
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group_arm_torso.setPlanningTime(20.0);
  bool success = bool(group_arm_torso.plan(my_plan));
  if (!success)
  {
      throw std::runtime_error("No plan found");
  }
  ROS_INFO_STREAM("Plan found for Motion in" << my_plan.planning_time_ << "seconds");

  /* --------- Execute the plan --------- */
  ros::Time strat = ros::Time::now();
  #if 1
  moveit::planning_interface::MoveItErrorCode e  = group_arm_torso.move(); 
  #else
  moveit::planning_interface::MoveItErrorCode e = group_arm_torso.asyncMove();
  #endif
  
  if(!bool(e))
  {
      throw std::runtime_error("Error execution plan");
  }
  else /* Motion Planning is completed */
  {
    
  }
      
}


void ROS_SUB::control(){

  ros::Rate r(100);

  while(ros::ok() && !done){

    cout<<"Des Pose Right: "<<endl;
    cout<<"X:"<<box_ref.Right_box_pose.position.x<<"   Y:"<<box_ref.Right_box_pose.position.y<<"   Z:"<<box_ref.Right_box_pose.position.x<<endl<<endl;
    cout<<"Des Pose Left: "<<endl;
    cout<<"X:"<<box_ref.Left_box_pose.position.x<<"   Y:"<<box_ref.Left_box_pose.position.y<<"   Z:"<<box_ref.Left_box_pose.position.x<<endl<<endl;
    
    if((box_ref.Right_box_pose.position.x!=0) && (box_ref.Right_box_pose.position.y!=0) && (box_ref.Right_box_pose.position.z!=0)&&(box_ref.Left_box_pose.position.x!=0) && (box_ref.Left_box_pose.position.y!=0) &&(box_ref.Left_box_pose.position.z!=0)){
      read=true;
    }

    if(read){

      both_arms_motion();
      done=true;
      
      }
      
      r.sleep();


  }

cout<<"\n";
cout << "Controller stopped" << endl;

}


/*
void ROS_SUB::control(){

  ros::Rate r(100);

  while(ros::ok() && !done_right && !done_left){

    cout<<"Des Pose Right: "<<endl;
    cout<<"X:"<<box_ref.Right_box_pose.position.x<<"   Y:"<<box_ref.Right_box_pose.position.y<<"   Z:"<<box_ref.Right_box_pose.position.x<<endl<<endl;
    
    if((box_ref.Right_box_pose.position.x!=0) && (box_ref.Right_box_pose.position.y!=0) &&(box_ref.Right_box_pose.position.z!=0)){
      read_right=true;
    }

    if(read_right){

      arm_right_motion();
      done_right=true;
      
      }

    cout<<"Des Pose Left: "<<endl;
    cout<<"X:"<<box_ref.Left_box_pose.position.x<<"   Y:"<<box_ref.Left_box_pose.position.y<<"   Z:"<<box_ref.Left_box_pose.position.x<<endl<<endl;
    
    if((box_ref.Left_box_pose.position.x!=0) && (box_ref.Left_box_pose.position.y!=0) &&(box_ref.Left_box_pose.position.z!=0)){
      read_left=true;
    }

    if(read_left){

      arm_left_motion();
      done_left=true;
      
      }
      
      r.sleep();

}

cout<<"\n";
cout << "Controller stopped" << endl;

}
*/

void ROS_SUB::run(){
      boost::thread control_thr( &ROS_SUB::control, this);
      ros::spin();      
}


int main(int argc, char** argv){   
   ros::init(argc,argv, "impedance_controller_node");
   ROS_SUB rs;
   rs.run();
   return 0;
}