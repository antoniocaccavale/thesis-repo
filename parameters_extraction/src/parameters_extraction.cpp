#include "parameters_extraction.h"
#include <XmlRpc.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

PARAM_EXTRACT::PARAM_EXTRACT(){

	joint_states_sub_ = nh_.subscribe("/joint_states", 1, &PARAM_EXTRACT::joint_states_cb, this);
  link_states_sub_ = nh_.subscribe("/gazebo/link_states", 1, &PARAM_EXTRACT::link_states_cb, this);
	
}

void PARAM_EXTRACT::joint_states_cb(sensor_msgs::JointState js){

   js_=js;

}

void PARAM_EXTRACT::link_states_cb(gazebo_msgs::LinkStates ls){

   ls_=ls;
   ee_l_=ls.twist[21];
   ee_r_=ls.twist[64];

}





void PARAM_EXTRACT::extract(){

  ros::Rate r(1000);

  //joints info of the total model
  std::vector<double> joint_position_min;
  std::vector<double> joint_position_max;
  std::vector<double> joint_vel_min;
  std::vector<double> joint_vel_max;
  std::vector<double> joint_damping;
  std::vector<double> joint_friction;
  std::vector<double> joint_max_effort;

//Get Model -> From URDF can obtain rbdl model, joint names, joint pos min and max, joint vel min and max, ...
  RigidBodyDynamics::Addons::URDFReadFromParamServer(
      &rbdl_model_, RigidBodyDynamics::FloatingBaseType::FixedBase, joint_names_,
      joint_position_min, joint_position_max, joint_vel_min, joint_vel_max,
      joint_damping, joint_friction, joint_max_effort);

 
  //Building two different Models: arm_left and arm_right

 
  arm_left_model_.AppendBody(*arm_left_model_.getModelData(), rbdl_model_.GetJointFrame(14), rbdl_model_.mJoints[14], rbdl_model_.mBodies[14], "arm_left_1_link");
  arm_left_model_.AppendBody(*arm_left_model_.getModelData(), rbdl_model_.GetJointFrame(15), rbdl_model_.mJoints[15], rbdl_model_.mBodies[15], "arm_left_2_link");
  arm_left_model_.AppendBody(*arm_left_model_.getModelData(), rbdl_model_.GetJointFrame(16), rbdl_model_.mJoints[16], rbdl_model_.mBodies[16], "arm_left_3_link");
  arm_left_model_.AppendBody(*arm_left_model_.getModelData(), rbdl_model_.GetJointFrame(17), rbdl_model_.mJoints[17], rbdl_model_.mBodies[17], "arm_left_4_link");
  arm_left_model_.AppendBody(*arm_left_model_.getModelData(), rbdl_model_.GetJointFrame(18), rbdl_model_.mJoints[18], rbdl_model_.mBodies[18], "arm_left_5_link");
  arm_left_model_.AppendBody(*arm_left_model_.getModelData(), rbdl_model_.GetJointFrame(19), rbdl_model_.mJoints[19], rbdl_model_.mBodies[19], "arm_left_6_link");
  arm_left_model_.AppendBody(*arm_left_model_.getModelData(), rbdl_model_.GetJointFrame(20), rbdl_model_.mJoints[20], rbdl_model_.mBodies[20], "arm_left_7_link");

  arm_right_model_.AppendBody(*arm_right_model_.getModelData(), rbdl_model_.GetJointFrame(21), rbdl_model_.mJoints[21], rbdl_model_.mBodies[21], "arm_right_1_link");
  arm_right_model_.AppendBody(*arm_right_model_.getModelData(), rbdl_model_.GetJointFrame(22), rbdl_model_.mJoints[22], rbdl_model_.mBodies[22], "arm_right_2_link");
  arm_right_model_.AppendBody(*arm_right_model_.getModelData(), rbdl_model_.GetJointFrame(23), rbdl_model_.mJoints[23], rbdl_model_.mBodies[23], "arm_right_3_link");
  arm_right_model_.AppendBody(*arm_right_model_.getModelData(), rbdl_model_.GetJointFrame(24), rbdl_model_.mJoints[24], rbdl_model_.mBodies[24], "arm_right_4_link");
  arm_right_model_.AppendBody(*arm_right_model_.getModelData(), rbdl_model_.GetJointFrame(25), rbdl_model_.mJoints[25], rbdl_model_.mBodies[25], "arm_right_5_link");
  arm_right_model_.AppendBody(*arm_right_model_.getModelData(), rbdl_model_.GetJointFrame(26), rbdl_model_.mJoints[26], rbdl_model_.mBodies[26], "arm_right_6_link");
  arm_right_model_.AppendBody(*arm_right_model_.getModelData(), rbdl_model_.GetJointFrame(27), rbdl_model_.mJoints[27], rbdl_model_.mBodies[27], "arm_right_7_link");

  
  //links and joints info of the two models
  std::vector<Body> left_bodies_;
  left_bodies_ = arm_left_model_.mBodies; //fields: mMass (double), mCenterOfMass (Math::Vector3d), mInertia (Math::Matrix3d)
  std::vector<Joint> left_joints_;
  left_joints_ = arm_left_model_.mJoints; //fields: Math::SpatialVector* mJointAxes, JointType 	mJointType, unsigned int 	mDoFCount, unsigned int 	q_index
  
  std::vector<Body> right_bodies_;
  right_bodies_ = arm_right_model_.mBodies; //fields: mMass (double), mCenterOfMass (Math::Vector3d), mInertia (Math::Matrix3d)
  std::vector<Joint> right_joints_;
  right_joints_ = arm_right_model_.mJoints; //fields: Math::SpatialVector* mJointAxes, JointType 	mJointType, unsigned int 	mDoFCount, unsigned int 

   
 // RigidBodyDynamics::ModelDatad model_data_ = *rbdl_model_.getModelData();


   while(ros::ok()){

     
  
    vector<double> temp1;
    temp1.resize(dofs_);
    vector<double> temp2;
    temp2.resize(dofs_);
    

    //Retrieving q_l_ (state of the left arm)
    js_.position.resize(rbdl_model_.dof_count);
    temp1.clear();
    for(int i=0; i<dofs_; i++){
      temp1.push_back(js_.position[i]);
    }
    Eigen::Map<VectorXd> q_l_(temp1.data(), temp1.size());
    
    //Retrieving q_r_ (state of the right arm)
    temp2.clear();
    for(int i=dofs_; i<2*dofs_; i++){
      temp2.push_back(js_.position[i]);
    }  
    Eigen::Map<VectorXd> q_r_(temp2.data(), temp2.size());

    

    //Retrieving qdot_l_ (velocity of the left arm)
    js_.velocity.resize(rbdl_model_.dof_count);
    temp1.resize(dofs_);
    temp1.clear();
    for(int i=0; i<dofs_; i++){
      temp1.push_back(js_.velocity[i]);
    }
    Eigen::Map<VectorXd> qdot_l_(temp1.data(), temp1.size());
    


    //Retrieving qdot_r_ (velocity of the right arm)
    temp2.resize(dofs_);
    temp2.clear();
    for(int i=dofs_; i<2*dofs_; i++){
      temp2.push_back(js_.velocity[i]);
    }  
    Eigen::Map<VectorXd> qdot_r_(temp2.data(), temp2.size());



    //Retrieving tau_l_ (torques applied to the left arm)
    js_.effort.resize(rbdl_model_.dof_count);
    temp1.resize(dofs_);
    temp1.clear();
    for(int i=0; i<dofs_; i++){
      temp1.push_back(js_.effort[i]);
    }
    Eigen::Map<VectorXd> tau_l_(temp1.data(), temp1.size());
    

    //Retrieving tau_r_ (torques applied to the right arm)
    temp2.resize(dofs_);
    temp2.clear();
    for(int i=dofs_; i<2*dofs_; i++){
      temp2.push_back(js_.effort[i]);
    }  
    Eigen::Map<VectorXd> tau_r_(temp2.data(), temp2.size());

    
    //Initialization of Jacobians, Inertia Matrices, Non Linear vectors and Accelerations
    MatrixXd J_l_;
    MatrixXd J_r_;
 //   MatrixXd G_l_;
 //   MatrixXd G_r_;
    J_l_.setZero(6, dofs_);
    J_r_.setZero(6, dofs_);
 //   G_l_.setZero(6, dofs_); 
 //   G_r_.setZero(6, dofs_);

    Math::MatrixNd H_l_;
    H_l_.setZero(dofs_, dofs_);
    Math::MatrixNd H_r_;
    H_r_.setZero(dofs_, dofs_);

    Math::VectorNd nonlin_l_;
    nonlin_l_.setZero(dofs_);
    Math::VectorNd nonlin_r_;
    nonlin_r_.setZero(dofs_);

    Math::VectorNd qddot_l_;
    qddot_l_.setZero(dofs_);
    Math::VectorNd qddot_r_;
    qddot_r_.setZero(dofs_);

    Eigen::Vector3d null_vec_;
    null_vec_.setZero();


  //  Eigen::Vector3d pos_ee_l_ = {ee_l_.position.x, ee_l_.position.y, ee_l_.position.z};
  //  Eigen::Vector3d pos_ee_r_ = {ee_r_.position.x, ee_r_.position.y, ee_r_.position.z};


    //Jacobians computation
    RigidBodyDynamics::CalcPointJacobian6D(arm_left_model_, q_l_, 7, null_vec_, J_l_, false);
    RigidBodyDynamics::CalcPointJacobian6D(arm_right_model_, q_r_, 7, null_vec_, J_r_, false);

 //   RigidBodyDynamics::CalcBodySpatialJacobian(arm_left_model_, *arm_left_model_.getModelData(), q_l_, 7, G_l_, false);
//    RigidBodyDynamics::CalcBodySpatialJacobian(arm_right_model_, *arm_right_model_.getModelData(), q_r_, 7, G_r_, false);

    //Inertia computation
    RigidBodyDynamics::CompositeRigidBodyAlgorithm (arm_left_model_, q_l_, H_l_, false);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm (arm_right_model_, q_r_, H_r_, false);

    //Non linear term computation
    RigidBodyDynamics::NonlinearEffects (arm_left_model_, *arm_left_model_.getModelData(),q_l_, qdot_l_, nonlin_l_);
    RigidBodyDynamics::NonlinearEffects (arm_right_model_, *arm_right_model_.getModelData(),q_r_, qdot_r_, nonlin_r_);

    //Acceleration computation
    RigidBodyDynamics::ForwardDynamics(arm_left_model_, q_l_, qdot_l_, tau_l_, qddot_l_, NULL);
    RigidBodyDynamics::ForwardDynamics(arm_right_model_, q_r_, qdot_r_, tau_r_, qddot_r_, NULL);  		


    //Printing Kinematics and Dynamics results

    cout<<"Point Jacobian left: "<<endl;
    cout<<"Num rows: "<<J_l_.rows()<<endl;
    cout<<"Num cols: "<<J_l_.cols()<<endl;
    for (int i = 0; i < J_l_.rows(); i++) {
      for (int j = 0; j < J_l_.cols(); j++) {
        std::cout <<"  "<<J_l_(i,j);
      }
    cout<<" "<<endl<<endl;
    }


    cout<<"Point Jacobian right: "<<endl;
    cout<<"Num rows: "<<J_r_.rows()<<endl;
    cout<<"Num cols: "<<J_r_.cols()<<endl;
    for (int i = 0; i < J_r_.rows(); i++) {
      for (int j = 0; j < J_r_.cols(); j++) {
        std::cout <<"  "<<J_r_(i,j);
      }
    cout<<" "<<endl<<endl;
    }
/*
    cout<<"Spatial Jacobian left: "<<endl;
    cout<<"Num rows: "<<G_l_.rows()<<endl;
    cout<<"Num cols: "<<G_l_.cols()<<endl;
    for (int i = 0; i < G_l_.rows(); i++) {
      for (int j = 0; j < G_l_.cols(); j++) {
          std::cout <<"  "<<G_l_(i,j);
      }
      cout<<"  "<<endl<<endl;
    }

    cout<<"Spatial Jacobian right: "<<endl;
    cout<<"Num rows: "<<G_r_.rows()<<endl;
    cout<<"Num cols: "<<G_r_.cols()<<endl;
    for (int i = 0; i < G_r_.rows(); i++) {
      for (int j = 0; j < G_r_.cols(); j++) {
          std::cout <<"  "<<G_r_(i,j);
      }
      cout<<"  "<<endl<<endl;
    }

*/
    cout<<"Inertia Matrix left: "<<endl;
    cout<<"Num rows: "<<H_l_.rows()<<endl;
    cout<<"Num cols: "<<H_l_.cols()<<endl;
    for (int i = 0; i < H_l_.rows(); i++) {
      for (int j = 0; j < H_l_.cols(); j++) {
          std::cout <<"  "<<H_l_(i,j);
      }
      cout<<"    "<<endl<<endl;
    }



    cout<<"Inertia Matrix right: "<<endl;
    cout<<"Num rows: "<<H_r_.rows()<<endl;
    cout<<"Num cols: "<<H_r_.cols()<<endl;
    for (int i = 0; i < H_r_.rows(); i++) {
      for (int j = 0; j < H_r_.cols(); j++) {
          std::cout <<"  "<<H_r_(i,j);
      }
      cout<<"  "<<endl<<endl;
    }
    
    cout<<"Nonlin left: "<<endl;
    for(int i=0; i<nonlin_l_.size(); i++){
        cout<<nonlin_l_[i]<<endl;
    }
    cout<<"   "<<endl<<endl;

    cout<<"Nonlin right: "<<endl;
    for(int i=0; i<nonlin_r_.size(); i++){
        cout<<nonlin_r_[i]<<endl;
    }
    cout<<"   "<<endl<<endl;

    cout<<"Acc left: "<<endl;
    for(int i=0; i<qddot_l_.size(); i++){
        cout<<qddot_l_[i]<<endl;
    }
    cout<<"   "<<endl<<endl;

    cout<<"Acc right: "<<endl;
    for(int i=0; i<qddot_r_.size(); i++){
        cout<<qddot_r_[i]<<endl;
    }
    cout<<" "<<endl<<endl;  


    //Concatenating the results (left + right: 7 + 7 = 14)

    //Total Generalized State Q
    VectorXd q_(q_l_.size() + q_r_.size());
    q_.setZero();
    q_ << q_l_, q_r_;

    cout<<"Tot Q: "<<endl;
    for(int i=0; i<q_.size(); i++){
        cout<<q_[i]<<endl;
    }
    cout<<" "<<endl<<endl;


    //Total Velocity
    VectorXd qdot_(qdot_l_.size() + qdot_r_.size());
    qdot_.setZero();
    qdot_ << qdot_l_, qdot_r_;

    cout<<"Tot Vel: "<<endl;
    for(int i=0; i<qdot_.size(); i++){
        cout<<qdot_[i]<<endl;
    }
    cout<<" "<<endl<<endl;


    //Total Acceleration
    VectorXd qddot_(qddot_l_.size() + qddot_r_.size());
    qddot_.setZero();
    qddot_ << qddot_l_, qddot_r_;

    cout<<"Tot Acc: "<<endl;
    for(int i=0; i<qddot_.size(); i++){
        cout<<qddot_[i]<<endl;
    }
    cout<<" "<<endl<<endl;


    //Total Torque
    VectorXd tau_(tau_l_.size() + tau_r_.size());
    tau_.setZero();
    tau_ << tau_l_, tau_r_;

    cout<<"Tot Tau: "<<endl;
    for(int i=0; i<tau_.size(); i++){
        cout<<tau_[i]<<endl;
    }
    cout<<" "<<endl<<endl;


    //Total Inertia Matrix
    MatrixXd H_;
    H_.setZero(dofs_*2, dofs_*2);

    H_.topLeftCorner(dofs_, dofs_) = H_l_;
    H_.bottomRightCorner(dofs_, dofs_) = H_r_;

    cout<<"Tot Inertia Matrix: "<<endl;
    cout<<"Num rows: "<<H_.rows()<<endl;
    cout<<"Num cols: "<<H_.cols()<<endl;
    for (int i = 0; i < H_.rows(); i++) {
      for (int j = 0; j < H_.cols(); j++) {
          std::cout <<"  "<<H_(i,j);
      }
      cout<<"  "<<endl<<endl;
    }

    //Total Non linear term
    VectorXd nonlin_(nonlin_l_.size() + nonlin_r_.size());
    nonlin_.setZero();
    nonlin_ << nonlin_l_, nonlin_r_;

    cout<<"Tot Nonlin term: "<<endl;
    for(int i=0; i<nonlin_.size(); i++){
        cout<<nonlin_[i]<<endl;
    }
    cout<<" "<<endl<<endl;


    //Compute left and right end effector poses with the retieved Jacobians
    Eigen::VectorXd x_l_;
    x_l_.resize(6);
    x_l_.setZero();
    Eigen::VectorXd x_dot_l_;
    x_dot_l_.resize(6);
    x_dot_l_.setZero();
    
    Eigen::VectorXd x_r_;
    x_r_.resize(6);
    x_r_.setZero();
    Eigen::VectorXd x_dot_r_;
    x_dot_r_.resize(6);
    x_dot_r_.setZero();



  	

    Eigen::VectorXd vel_l_;
    //Math::SpatialVector vel_r_;
    vel_l_.resize(6);
    vel_l_.setZero();
    vel_l_ = CalcPointVelocity6D(arm_left_model_,q_l_,qdot_l_,7,null_vec_,false);
    

  //  Eigen::Vector3d lin_vel_l_, pos_l_;
  //  lin_vel_l_.setZero();
  //  lin_vel_l_.resize(3);
  //  lin_vel_l_={vel_l_[3], vel_l_[4], vel_l_[5]};

  //  pos_l_.resize(3);
  //  pos_l_.setZero();
  //  pos_l_=CalcBodyToBaseCoordinates(arm_left_model_, q_l_, 7, null_vec_, false); 
    //pos_l_=lin_vel_l_;
    //pos_l_+=lin_vel_l_*(1/1000);
  //  cout<<"Left pos: "<<endl; 
  //  cout<<" "<<pos_l_<<endl<<endl;

    
    

    Eigen::VectorXd vel_r_;
    //Math::SpatialVector vel_r_;
    vel_r_.resize(6);
    vel_r_.setZero();
    vel_r_ = CalcPointVelocity6D(arm_right_model_,q_r_,qdot_r_,7,null_vec_,false);
  //  Eigen::Vector3d lin_vel_r_, pos_r_;
  //  lin_vel_r_={vel_r_[3], vel_r_[4], vel_r_[5]};
  //  pos_r_=CalcBodyToBaseCoordinates(arm_right_model_, q_r_, 7, null_vec_, false); 
   // pos_r_=lin_vel_r_;
  //  pos_r_+=lin_vel_r_*(1/1000);
  //  cout<<"Right pos: "<<endl; 
  //  cout<<" "<<pos_r_<<endl<<endl; 


    double dt = 1/1000; // for integration (inverse of rate)
    
    
    //Left end effector pose computation
    x_dot_l_ = J_l_*qdot_l_;
    //pose_l_ += pose_l_*dt;
    cout<<" Computed left ee twist: "<<endl; 
    cout<<" "<<x_dot_l_<<endl<<endl;

    //Comparision (point Jacobian, just for the position I suppose)
    cout<<"Original left ee twist: "<<endl;
    cout<<" "<<vel_l_<<endl<<endl;


    //Right end effector pose computation 
    x_dot_r_ = J_r_*qdot_r_;
    //pose_r_ += pose_r_*dt;
    cout<<" Computed right ee twist: "<<endl;
    cout<<" "<<x_dot_r_<<endl<<endl;
    

    //Comparision (point Jacobian, just for the position I suppose)
    cout<<"Original right ee twist: "<<endl;
    cout<<" "<<vel_r_<<endl<<endl;

  
    //Computed acc
    Eigen::VectorXd acc_l_;
    acc_l_.resize(dofs_);
    acc_l_.setZero();

    acc_l_=H_l_.inverse()*(-nonlin_l_+tau_l_);
    
    //Computed left acceleration:
    cout<<" Computed left acc: "<<endl; 
    cout<<" "<<acc_l_<<endl<<endl;

    //Original left acceleration
    cout<<"Original left acc: "<<endl;
    cout<<" "<<qddot_l_<<endl<<endl;


    Eigen::VectorXd acc_r_;
    acc_r_.resize(dofs_);
    acc_r_.setZero();

    acc_r_=H_r_.inverse()*(-nonlin_r_+tau_r_);
    
    //Computed left acceleration:
    cout<<" Computed right acc: "<<endl; 
    cout<<" "<<acc_r_<<endl<<endl;

    //Original left acceleration
    cout<<"Original right acc: "<<endl;
    cout<<" "<<qddot_r_<<endl<<endl;



    r.sleep();

  }

     
  cout<<"\n";
  cout << "PARAM_EXTRACT finishes" << endl;

}




void PARAM_EXTRACT::run(){
      boost::thread extract_thr( &PARAM_EXTRACT::extract, this);
      ros::spin();      
}



int main(int argc, char** argv){   
   ros::init(argc,argv, "parameters_extraction_node");
   PARAM_EXTRACT par_extr;
   par_extr.run();
   return 0;
}
