/*
 * Copyright 2018 Timothy Sandy
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

#ifndef INCLUDE_CONFUSION_ERTMODULEROS_H_
#define INCLUDE_CONFUSION_ERTMODULEROS_H_

#include <confusion_ros/OdomWithState.h>
#include <confusion_ros/TagArray.h>
#include <confusion/modules/external_reference_tracking/ExternalReferenceTrackingModule.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "confusion_ros/ert_ros_utilities.h"
#include "confusion_ros/ros_conversions.h"

namespace confusion {

class ExternalReferenceTrackingModuleRos
    : public ExternalReferenceTrackingModule {
 public:
  ExternalReferenceTrackingModuleRos(ros::NodeHandle& node, ConFusor& conFusor,
                                     const boost::property_tree::ptree& pt,
                                     int poseMeasIndex,
                                     bool* newMeasReceivedFlag);

  void initializeSensorframeOffserDerived(
      const std::pair<const std::string, boost::property_tree::ptree>& subTree)
      final;

  template <typename MsgType>
  void externalPoseMeasCallback(const MsgType& msg);

  void externalPoseMeasTFStampedCallback(
      const geometry_msgs::TransformStampedPtr& msg);

  void externalPoseMeasOdomCallback(const nav_msgs::OdometryPtr& msg);

  void externalPoseMeasOdomWithStateCallback(
      const confusion::OdomWithStatePtr& msg);

  void resetSensorCallback(const std_msgs::StringPtr& msg);

  void publishFrameOffsetTfs();

 private:
  void processPoseMeasBeforeAssigningToStateDerived(
      const std::shared_ptr<ErtMeasBase>& ertMeasPtr, const double& t,
      const Pose<double>& T_w_body) override;

  ros::NodeHandle& node_;
  std::vector<ros::Subscriber> subsExternalPoseMeas_;

  tf::TransformBroadcaster tfBroadcaster_;
  ros::Subscriber resetSensorSub_;
};

}  // namespace confusion

#endif /* INCLUDE_CONFUSION_ERTMODULEROS_H_ */
