/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the author nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Marcus Ebner */
#include <iimoveit/robot_interface.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include <keyboard/Key.h>
//#include <Vector3.h>
//#include <Transform.h>



//Code for Clutching=====================================================
std_msgs::Bool declutch;
bool declutch_uplatch;
bool declutch_downlatch;
bool declutch_status;

void chatterCallbackUp(const keyboard::Key& msg)
{
    declutch.data = false;
}

void chatterCallbackDown(const keyboard::Key& msg)
{
    declutch.data = true;
}

geometry_msgs::PoseStamped declutch_pose;
//geometry_msgs::PoseStamped debug_pose;
//=======================================================================




//TODO not only use positions, use speed and accelerations too
//TODO try out using moveIt! planning live

namespace pose_follower {


    class PoseFollower : public iimoveit::RobotInterface {
    public:
        PoseFollower(ros::NodeHandle* node_handle, const std::string& planning_group, const std::string& base_frame, double scale_x, double scale_y, double scale_z, double scale_rot_x, double scale_rot_y, double scale_rot_z, double max_radius)
                : RobotInterface(node_handle, planning_group, base_frame),
                  scale_x_(scale_x),
                  scale_y_(scale_y),
                  scale_z_(scale_z),
                  scale_rot_x_(scale_rot_x),
                  scale_rot_y_(scale_rot_y),
                  scale_rot_z_(scale_rot_z),
                  max_radius_(max_radius),
                  max_radius2_(max_radius*max_radius),
                  first_time_(true) {

            iiwa_initial_joint_positions_.joint_names.resize(7);
            //iwa_initial_joint_positions_.joint_names = RobotInterface::getJointNames();
            iiwa_initial_joint_positions_.joint_names[0] = "iiwa_joint_1";
            iiwa_initial_joint_positions_.joint_names[1] = "iiwa_joint_2";
            iiwa_initial_joint_positions_.joint_names[2] = "iiwa_joint_3";
            iiwa_initial_joint_positions_.joint_names[3] = "iiwa_joint_4";
            iiwa_initial_joint_positions_.joint_names[4] = "iiwa_joint_5";
            iiwa_initial_joint_positions_.joint_names[5] = "iiwa_joint_6";
            iiwa_initial_joint_positions_.joint_names[6] = "iiwa_joint_7";
            iiwa_initial_joint_positions_.points.resize(1);
            iiwa_initial_joint_positions_.points[0].positions.resize(7);

            // Anthropomorphic
            iiwa_initial_joint_positions_.points[0].positions[0] = 3.1416/180.0 * -1.0 * -30.97;
            iiwa_initial_joint_positions_.points[0].positions[1] = 3.1416/180.0 * (-1.0 * 18.34 + 90.0);
            iiwa_initial_joint_positions_.points[0].positions[2] = 3.1416/180.0 * -21.67;
            iiwa_initial_joint_positions_.points[0].positions[3] = 3.1416/180.0 * -1.0 * -57.57;
            iiwa_initial_joint_positions_.points[0].positions[4] = 3.1416/180.0 * (59.36 - 90.0);
            iiwa_initial_joint_positions_.points[0].positions[5] = 3.1416/180.0 * (-1.0 * -4.63 + 90.0);
            iiwa_initial_joint_positions_.points[0].positions[6] = 3.1416/180.0 * 0.0;


            operator_rotation_ = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);

        }

        void moveToBasePose() {
            planAndMove(base_pose_, std::string("base pose"));
        }

        void registerSubscriberRelative(const std::string& topic) {
            pose_subscriber_ = node_handle_->subscribe(topic, 1, &PoseFollower::poseCallbackRelative, this);
        }

        void registerSubscriberAbsolute(const std::string& topic) {
            pose_subscriber_ = node_handle_->subscribe(topic, 1, &PoseFollower::poseCallbackAbsolute, this);
        }

        void setBasePose(const geometry_msgs::Pose& pose) {
            base_pose_ = pose;
        }

        geometry_msgs::Pose getBasePose() {
            return base_pose_;
        }

        void moveToInitialJointPositions() {
            planAndMove(iiwa_initial_joint_positions_.points[0].positions, std::string("initial joint positions"));
        }

        void setBasePoseToCurrent() {
            base_pose_ = getPose(std::string("iiwa_s_model_finger_1")).pose; // formerly: "iiwa_link_ee"
            //target_pose = base_pose_;
            std::cout<<"setBasePoseToCurrent"<<std::endl;
        }

        void setTargetPoseToBase() {
            std::cout<<"setTargetPoseToBase"<<std::endl;
            target_pose = base_pose_;
            declutch_pose.pose = base_pose_;
            //omega_pose_old.pose =

        }

        void setScalingFactors(double scalefactor_trans_x, double scalefactor_trans_y, double scalefactor_trans_z, double scalefactor_rot_x, double scalefactor_rot_y, double scalefactor_rot_z){
            scale_x_ = scalefactor_trans_x;
            scale_y_ = scalefactor_trans_y;
            scale_z_ = scalefactor_trans_z;
            scale_rot_x_ = scalefactor_rot_x;
            scale_rot_y_ = scalefactor_rot_y;
            scale_rot_z_ = scalefactor_rot_z;
        }
        void calculateRelativePose(geometry_msgs::PoseStamped &old_pose, geometry_msgs::PoseStamped &new_pose, geometry_msgs::PoseStamped &relative_pose){
            //translational part
            relative_pose.pose.position.x = new_pose.pose.position.x - old_pose.pose.position.x;
            relative_pose.pose.position.y = new_pose.pose.position.y - old_pose.pose.position.y;
            relative_pose.pose.position.z = new_pose.pose.position.z - old_pose.pose.position.z;
            //TODO insert rotational part!?
        }


        // Own implementation of a (only rotational) transform, because transformPose() from TransformListener throws extrapolation exception
        geometry_msgs::PoseStamped transformOperatorPose(const geometry_msgs::PoseStamped pose_to_transform) {
            geometry_msgs::PoseStamped transformed_pose;
            tf::Vector3 position_vec = tf::Vector3(pose_to_transform.pose.position.x, pose_to_transform.pose.position.y, pose_to_transform.pose.position.z);

            transformed_pose = pose_to_transform;
            return transformed_pose;
        }

    private:
        ros::Subscriber pose_subscriber_;
        trajectory_msgs::JointTrajectory iiwa_initial_joint_positions_;
        geometry_msgs::Pose base_pose_;
        double scale_x_;
        double scale_y_;
        double scale_z_;
        double scale_rot_x_;
        double scale_rot_y_;
        double scale_rot_z_;
        double max_radius_;
        double max_radius2_;
        bool first_time_;
        tf::Quaternion omega_old_quaternion_; // relation between initial poses of iiwa and omega
        double omega_x_init_, omega_y_init_, omega_z_init_;
        // tf::Transform operator_frame_;
        // tf::TransformBroadcaster transform_broadcaster_;
        // tf::TransformListener transform_listener_;
        // tf::StampedTransform operator_transform_;
        tf::Quaternion operator_rotation_;
        geometry_msgs::PoseStamped omega_pose_new;
        geometry_msgs::PoseStamped omega_pose_relative;
        geometry_msgs::PoseStamped omega_pose_old;
        geometry_msgs::PoseStamped omega_calibpose;
        tf::Pose omega_pose_new_p;
        tf::Pose omega_pose_relative_p;
        tf::Pose omega_pose_old_p;
        tf::Pose target_pose_p;
        tf::Pose base_pose_p;
        tf::Pose omega_calibpose_p;
        tf::Vector3 scaled_trans;
        double euler_x, euler_y, euler_z;
        //new euler variables for adjusting the coordinate frame of omega to the endeffector frame
        double euler_x_new, euler_y_new, euler_z_new;
        tfScalar trans_x_scaled, trans_y_scaled, trans_z_scaled;
        //tf::Quaternion omega_old_quaternion;
        //tf::Quaternion omega_new_quaternion;
        //tf::Quaternion target_quaternion;
        tf::Quaternion relative_quaternion;
        tf::Quaternion relative_quaternion_scaled;
        tf::Quaternion omega_old_inverse_quaternion;

        geometry_msgs::Pose target_pose;
        //initialize the scaling factors



        void poseCallbackRelative(const geometry_msgs::PoseStamped::ConstPtr& msg) {

            omega_pose_new = transformOperatorPose(*msg); //Why??? Why not omega_pose_new = msg ?
            
            if(first_time_) {
                base_pose_ = target_pose;
                tf::poseMsgToTF(base_pose_, base_pose_p);
                omega_calibpose = omega_pose_new;
                tf::poseMsgToTF(omega_calibpose.pose, omega_calibpose_p);
                //target_pose = declutch_pose.pose;
                std::cout<< "first time executed"<< std::endl;
                first_time_ = false;
            }

            //std::cout<< "Immina Callback1"<<std::endl;
            /*if (first_time_){
              omega_pose_old.pose = base_pose_;
              first_time_ = false;
            }*/
            //ROS_INFO("omega old x is ");

            //convert geometry_msgs::Pose to tf::Pose for following calculations================
            tf::poseMsgToTF(omega_pose_new.pose, omega_pose_new_p);                           // 
            //tf::poseMsgToTF(omega_pose_old.pose, omega_pose_old_p);                           //
            //==================================================================================  
            std::cout<<"x of omega new is "<< omega_pose_new_p.getOrigin().getX()<<std::endl;
            std::cout<<"x of inverse of calib is "<< omega_calibpose_p.getOrigin().getX()<<std::endl;



            //calculate omega_pose_relative ====================================================
            omega_pose_relative_p = omega_pose_new_p * omega_calibpose_p.inverse();            //
            //==================================================================================
            std::cout<<"x of omega relative is "<< omega_pose_relative_p.getOrigin().getX()<<std::endl;



            //initialise omega_pose_old for the next iteration==================================
            //omega_pose_old = omega_pose_new;                                                  //
            //==================================================================================
            std::cout<<"x before scale is "<< omega_pose_relative_p.getOrigin().getX()<<std::endl;

            
            //scale omega_pose_relative ========================================================
            //scale the translational part
            scaled_trans = omega_pose_relative_p.getOrigin();
            std::cout<< "x ist "<< scaled_trans.getX() << std::endl;
            trans_y_scaled = scaled_trans.getY() * scale_y_;
            trans_z_scaled = scaled_trans.getZ() * scale_z_;
            trans_x_scaled = scaled_trans.getX() * scale_x_;
            //std::cout<< "x nachher "<< trans_x_scaled << std::endl;
            scaled_trans.setX(trans_x_scaled);
            scaled_trans.setY(trans_y_scaled);
            scaled_trans.setZ(trans_z_scaled);
            omega_pose_relative_p.setOrigin(scaled_trans);
            //scale the rotational part
            relative_quaternion = omega_pose_relative_p.getRotation();
            tf::Matrix3x3 rotMatrix(relative_quaternion);
            rotMatrix.getEulerYPR(euler_z, euler_y, euler_x);
            euler_x_new = euler_x * scale_rot_x_;
            euler_y_new = euler_y * scale_rot_y_;
            euler_z_new = euler_z * scale_rot_z_;
            rotMatrix.setEulerYPR(euler_z_new, euler_y_new, euler_x_new);
            rotMatrix.getRotation(relative_quaternion_scaled);
            omega_pose_relative_p.setRotation(relative_quaternion_scaled);
            //==================================================================================
            std::cout<<"x after scale is "<< omega_pose_relative_p.getOrigin().getX()<<std::endl;

            //Transformation of target pose=====================================================
            //conversion to TF necessary for transformation
            tf::poseMsgToTF(target_pose, target_pose_p);
            target_pose_p = base_pose_p * omega_pose_relative_p;
            tf::poseTFToMsg(target_pose_p, target_pose);
            //==================================================================================

            
            //tf::Quaternion target_quaternion(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
            //std::cout << "x of target quaternion is " << target_quaternion.getX() << std::endl;
            //target_quaternion *= relative_quaternion_scaled;
            //std::cout << "target pose orientation x is " << target_pose.orientation.x << std::endl;
            


            //Clutching Algorithm=====================================================
            if(declutch_downlatch) {
                //target_pose = declutch_pose.pose;
                first_time_ = true;
                target_pose = declutch_pose.pose;
                declutch_downlatch = false;
                declutch_status = false;
            }

            if(declutch_uplatch){
                declutch_pose.pose = target_pose;
               std::cout<< "Callback CAllback Calllback declutch_uplatch is true"<<std::endl;
               declutch_uplatch = false;
               declutch_status = true;
            } 
            if(declutch_status)publishPoseGoal(declutch_pose, 0.01);
            
            if(!declutch_status) publishPoseGoal(target_pose, 0.01);
            //========================================================================

        }

        void poseCallbackAbsolute(const geometry_msgs::PoseStamped::ConstPtr& msg) {
            publishPoseGoal(msg->pose, 0.01);
        }
    };
} // namespace pose_follower

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_pose_follower");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ros::Publisher pose_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("getPoseDebug", 1);

    ros::Subscriber keyup_sub = node_handle.subscribe("/keyboard/keyup", 1, &chatterCallbackUp);
    ros::Subscriber keydown_sub = node_handle.subscribe("/keyboard/keydown", 1, &chatterCallbackDown);

    spinner.start();

    double scale_x, scale_y, scale_z, scale_rot_x, scale_rot_y, scale_rot_z;
    node_handle.param("/iiwa/pose_follower/scale_x", scale_x, 1.0);
    node_handle.param("/iiwa/pose_follower/scale_y", scale_y, 1.0);
    node_handle.param("/iiwa/pose_follower/scale_z", scale_z, 1.0);
    node_handle.param("/iiwa/pose_follower/scale_rot_x", scale_rot_x, 1.0);
    node_handle.param("/iiwa/pose_follower/scale_rot_y", scale_rot_y, 1.0);
    node_handle.param("/iiwa/pose_follower/scale_rot_z", scale_rot_z, 1.0);
    bool udp_input;
    node_handle.param("/iiwa/pose_follower/udp", udp_input, false);



    pose_follower::PoseFollower pose_follower(&node_handle, "manipulator", "world", scale_x, scale_y, scale_z, scale_rot_x, scale_rot_y, scale_rot_z, 2);

    pose_follower.setScalingFactors(1, 1, 1, 0.05, 0.05, 0.05);
    //pose_follower.waitForApproval();

    pose_follower.moveToInitialJointPositions();
    pose_follower.setBasePoseToCurrent();
    pose_follower.setTargetPoseToBase();

   

    if(udp_input) {
        pose_follower.registerSubscriberRelative(std::string("/poseFromUDP/PoseStamped"));
        ROS_INFO_NAMED("pose_follower", "Subscribed to pose from UDP!");
    }
    else {
        pose_follower.registerSubscriberRelative(std::string("/poseFromFile/PoseStampedRelative"));
        //pose_follower.registerSubscriberAbsolute(std::string("/poseFromFile/PoseStampedAbsolute"));
        ROS_INFO_NAMED("pose_follower", "Subscribed to pose from file!");
    }


    bool declutch_lag = false;

    declutch.data = false;
    declutch_uplatch = false;
    declutch_downlatch = false;
    declutch_status = false;

    spinner.stop();
    ros::Rate rate(30);

    while(ros::ok()) {
        ros::spinOnce();
        if (declutch_lag-declutch.data == -1)declutch_uplatch = true;
        if (declutch.data-declutch_lag == -1)declutch_downlatch = true;
        declutch_lag = declutch.data;
        //ros::spinOnce();        
        if(declutch_uplatch) std::cout<< "declutch_uplatch is true"<<std::endl;
        if(declutch_status) std::cout<< "declutch is true"<<std::endl;
        if(declutch_downlatch) std::cout<< "declutch_downlatch is true"<<std::endl;
        //std::cout<< "Im in da while man"<<std::endl;
        
        rate.sleep();
        //declutch_uplatch = false;
        //declutch_downlatch = false;

    }
    //spinner.stop();

    ros::shutdown();
    return 0;
}
