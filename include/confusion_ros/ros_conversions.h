/*
 * Copyright 2018 ETH Zurich, Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_CONFUSION_ROSCONVERSIONS_H_
#define INCLUDE_CONFUSION_ROSCONVERSIONS_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>

#include <Eigen/Dense>
#include <iostream>

#include "confusion/models/ImuMeas.h"
#include "confusion/utilities/Pose.h"

namespace confusion {

geometry_msgs::Vector3 VectorToRosMsg(const Eigen::Vector3d& vector);

Eigen::Vector3d RosMsgToVector(const geometry_msgs::Vector3& msg_point);

geometry_msgs::Quaternion QuaternionToRosMsg(const Eigen::Quaterniond& quat);

Eigen::Quaterniond RosMsgToQuaternion(
    const geometry_msgs::Quaternion& msg_quat);

geometry_msgs::TransformStamped getTransformStampedMsg(
    Pose<double> T, std::string frame_id, std::string child_frame_id,
    ros::Time stamp);

geometry_msgs::Pose getMsg(Pose<double> T);

geometry_msgs::PoseStamped getMsg(Pose<double> T, std::string frame_id,
                                  ros::Time stamp);

geometry_msgs::PoseStamped getMsg(Pose<double> T, std::string frame_id);

Pose<double> getPoseFromMsg(geometry_msgs::Pose msg);
Pose<double> getPoseFromMsg(geometry_msgs::Transform msg);

Pose<double> getPoseFromMsg(const tf::Transform& msg);
Pose<double> getPoseFromMsg(const tf::StampedTransform& msg);

Pose<double> getPoseFromMsg(const geometry_msgs::TransformStampedPtr& msg);

Pose<double> getPoseFromMsg(const nav_msgs::OdometryPtr& msg);

tf::StampedTransform getTfStampedMsg(Pose<double> T_a_b,
                                     std::string parentFrameId,
                                     std::string childFrameId);

tf::Transform getTfMsg(Pose<double> T_a_b);

nav_msgs::Odometry getOdometryMsg(double t, Pose<double> T_w_i,
                                  Eigen::Vector3d angVel,
                                  Eigen::Vector3d linVel);

// todo Also output a timestamp somehow?
std_msgs::Float64MultiArray matrixToRosMsg(std::string topic,
                                           Eigen::MatrixXd mat,
                                           std::string label = "");

std_msgs::Float64MultiArray vectorToRosMsg(std::string topic,
                                           Eigen::VectorXd vec, double stamp,
                                           std::string label = "");

/**
 * Convert a ROS IMU message to ConFusion type. Note that this does not fill in
 * the calibration struct or gravity orientation parameter reference.
 * @param msg The ROS message
 * @return The ConFusion-type IMU measurement
 */
ImuMeas rosmsgToConfusionImuMeas(const sensor_msgs::Imu& msg);

/**
 * Convert a ROS IMU message to ConFusion type. Note that this does not fill in
 * the calibration struct or gravity orientation parameter reference.
 * @param meas The IMU measurement
 * @return The ROS message
 */
sensor_msgs::Imu confusionImuMeasToRosmsg(const ImuMeas& meas);

}  // namespace confusion

#endif /* INCLUDE_CONFUSION_ROSCONVERSIONS_H_ */
