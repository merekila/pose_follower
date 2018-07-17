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
#include <tf/transform_broadcaster.h>
#include <stdio.h>
#include "std_msgs/Bool.h"
#include <keyboard/Key.h>


//Code for Clutching=====================================================
std_msgs::Bool declutch;
bool declutch_uplatch;
bool declutch_downlatch;

void chatterCallbackUp(const keyboard::Key& msg)
{
    declutch.data = false;
}

void chatterCallbackDown(const keyboard::Key& msg)
{
    declutch.data = true;
}

geometry_msgs::PoseStamped declutch_pose;
geometry_msgs::PoseStamped debug_pose;
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
        double euler_x, euler_y, euler_z;
        //tf::Quaternion omega_old_quaternion;
        //tf::Quaternion omega_new_quaternion;
        //tf::Quaternion target_quaternion;
        tf::Quaternion relative_quaternion;
        tf::Quaternion relative_quaternion_scaled;
        tf::Quaternion omega_old_inverse_quaternion;

        geometry_msgs::Pose target_pose;
        //initialize the scaling factors

        /*    scale_rot_x_ = 0.2;
        scale_rot_y_ = 0.2;
        scale_rot_z_ = 0.2;
        scale_x_ = 3;
        scale_y_ = 3;
        scale_z_ = 3;*/

        double scaled_trans_x;
        double scaled_trans_y;
        double scaled_trans_z;

        void poseCallbackRelative(const geometry_msgs::PoseStamped::ConstPtr& msg) {

            //omega_pose_old = last_pose;
            omega_pose_new = transformOperatorPose(*msg); //Why??? Why not omega_pose_new = msg ?
            if(first_time_) {

                omega_pose_old = omega_pose_new;
                //std::cout<< "omega pose old set to new!"<< std::endl;
                first_time_ = false;
            }

            //std::cout<< "Immina Callback1"<<std::endl;
            /*if (first_time_){
              omega_pose_old.pose = base_pose_;
              first_time_ = false;
            }*/
            //ROS_INFO("omega new x is zero ", omega_pose_new.pose.position.x);
            //ROS_INFO("omega old x is ");

            //calculate omega_pose_relative ====================================================
            //translational part
            calculateRelativePose(omega_pose_old, omega_pose_new, omega_pose_relative);
            //rotational part
            tf::Quaternion omega_old_quaternion(omega_pose_old.pose.orientation.x, omega_pose_old.pose.orientation.y, omega_pose_old.pose.orientation.z, omega_pose_old.pose.orientation.w);
            tf::Quaternion omega_new_quaternion(omega_pose_new.pose.orientation.x, omega_pose_new.pose.orientation.y, omega_pose_new.pose.orientation.z, omega_pose_new.pose.orientation.w);
            std::cout << "x of omega old quaternion is " << omega_old_quaternion.getX() << std::endl;
            std::cout << "x of omega new quaternion is " << omega_new_quaternion.getX() << std::endl;

            omega_old_inverse_quaternion = inverse(omega_old_quaternion);
            std::cout << "x of omega old quaternion inverse is " << omega_old_inverse_quaternion.getX() << std::endl;

            relative_quaternion = omega_old_inverse_quaternion * omega_new_quaternion;
            std::cout << "x of relative quaternion is " << relative_quaternion.getX() << std::endl;

            /*
            omega_pose_relative.pose.orientation.x = relative_quaternion.getX();
            omega_pose_relative.pose.orientation.y = relative_quaternion.getY();
            omega_pose_relative.pose.orientation.z = relative_quaternion.getZ();
            omega_pose_relative.pose.orientation.w = relative_quaternion.getW();
            */
            //==================================================================================



            //initialise omega_pose_old for the next iteration==================================
            /*omega_pose_old.pose.position.x = omega_pose_new.pose.position.x;
            omega_pose_old.pose.position.y = omega_pose_new.pose.position.y;
            omega_pose_old.pose.position.z = omega_pose_new.pose.position.z;*/
            omega_pose_old = omega_pose_new;
            std::cout << "omega pose old position x is " << omega_pose_old.pose.position.x << std::endl;


            //std::cout<< "Immina Callback2"<<std::endl;


            //scale omega_pose_relative ========================================================


            //scale the translational part
            scaled_trans_x = omega_pose_relative.pose.position.x * scale_x_;
            scaled_trans_y = omega_pose_relative.pose.position.y * scale_y_;
            scaled_trans_z = omega_pose_relative.pose.position.z * scale_z_;
            //std::cout << "scaled_trans_x is " << scaled_trans_x << std::endl;

            //scale the rotational part
            //std::cout << "x of relative quaternion is " << relative_quaternion.getX() << std::endl;

            tf::Matrix3x3 rotMatrix(relative_quaternion);
            rotMatrix.getEulerYPR(euler_z, euler_y, euler_x);
            std::cout << "euler_x is " << euler_x << std::endl;
            euler_x *= scale_rot_x_;
            euler_y *= scale_rot_y_;
            euler_z *= scale_rot_z_;
            std::cout << "euler_x is " << euler_x << std::endl;
            //std::cout << "scale rot x is " << scale_rot_x_ << std::endl;
            rotMatrix.setEulerYPR(euler_z, euler_y, euler_x);
            rotMatrix.getRotation(relative_quaternion_scaled);
            //==================================================================================
            std::cout << "target pose orientation x before transform is " << target_pose.orientation.x << std::endl;


            tf::Quaternion target_quaternion(target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w);
            std::cout << "x of target quaternion is " << target_quaternion.getX() << std::endl;
            target_quaternion *= relative_quaternion_scaled;

            //std::cout<< " scaled x is "<< scaled_trans_x <<" scaled y is "<< scaled_trans_y <<" scaled z is "<< scaled_trans_z <<std::endl;
            //Add relative pose to the last target
            target_pose.position.x += scaled_trans_x;
            target_pose.position.y += scaled_trans_y;
            target_pose.position.z += scaled_trans_z;
            //target_pose.orientation *= relative_quaternion;
            target_pose.orientation.x = target_quaternion.getX();
            target_pose.orientation.y = target_quaternion.getY();
            target_pose.orientation.z = target_quaternion.getZ();
            target_pose.orientation.w = target_quaternion.getW();
            std::cout << "target pose orientation x is " << target_pose.orientation.x << std::endl;
            //std::cout << "target pose pose x is " << target_pose.position.x << std::endl;
            //base_pose_.orientation = target_pose.orientation;
            //omega_pose_old.pose = target_pose;


            //Clutching Algorithm=====================================================
            if(declutch_downlatch) {
                base_pose_ = target_pose;
                first_time_ = true;
                omega_pose_old = omega_pose_new;

             /*   omega_pose_old.pose.position.x = omega_pose_new.pose.position.x;
                omega_pose_old.pose.position.y = omega_pose_new.pose.position.y;
                omega_pose_old.pose.position.z = omega_pose_new.pose.position.z;*/
            }
            if(declutch_uplatch) declutch_pose.pose = target_pose;
            if(declutch.data) publishPoseGoal(declutch_pose, 0.01);
            if(!declutch.data) publishPoseGoal(target_pose, 0.01);
            //========================================================================

            /*
            //Adjust omega_pose_old
            omega_pose_old.pose.position.x = target_pose.position.x - base_pose_.pose.x;
            omega_pose_old.pose.position.y = target_pose.position.y - base_pose_.pose.y;
            omega_pose_old.pose.position.z = target_pose.position.z - base_pose_.pose.z;

            //todo adjust quaternions of omega_pose_old
            tf::Quaternion omega_old_quaternion();
            */
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

    ros::Subscriber keyup_sub = node_handle.subscribe("/keyboard/keyup", 10, &chatterCallbackUp);
    ros::Subscriber keydown_sub = node_handle.subscribe("/keyboard/keydown", 10, &chatterCallbackDown);

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



    pose_follower.moveToInitialJointPositions();
    pose_follower.setBasePoseToCurrent();
    pose_follower.setTargetPoseToBase();
    pose_follower.setScalingFactors(3, 3, 3, 0.2, 0.2, 0.2);

    //pose_follower.waitForApproval();

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


    ros::Rate rate(30);
    spinner.stop();
    while(ros::ok()) {
        ros::spinOnce();
        if (declutch_lag-declutch.data == -1)declutch_uplatch = true;
        if (declutch.data-declutch_lag == -1)declutch_downlatch = true;
        declutch_lag = declutch.data;
        if(declutch_uplatch) std::cout<< "declutch_uplatch is true"<<std::endl;
        if(declutch.data) std::cout<< "declutch is true"<<std::endl;
        if(declutch_downlatch) std::cout<< "declutch_downlatch is true"<<std::endl;
        //std::cout<< "Im in da while man"<<std::endl;
        ros::spinOnce();
        rate.sleep();
        declutch_uplatch = false;
        declutch_downlatch = false;

    }

    ros::shutdown();
    return 0;
}
