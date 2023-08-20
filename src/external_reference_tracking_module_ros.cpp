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

#include "confusion_ros/external_reference_tracking_module_ros.h"

namespace confusion {

ExternalReferenceTrackingModuleRos::ExternalReferenceTrackingModuleRos(
    ros::NodeHandle& node, ConFusor& conFusor,
    const boost::property_tree::ptree& pt, int poseMeasIndex,
    bool* newMeasReceivedFlag)
    : ExternalReferenceTrackingModule(conFusor, pt, poseMeasIndex,
                                      newMeasReceivedFlag),
      node_(node) {
  resetSensorSub_ = node_.subscribe(
      "/reset_ert_sensor", 10,
      &ExternalReferenceTrackingModuleRos::resetSensorCallback, this);
}

void ExternalReferenceTrackingModuleRos::initializeSensorframeOffserDerived(
    const std::pair<const std::string, boost::property_tree::ptree>& subTree) {
  // Check for msg types and initialize subscribers.
  std::string msg_type = subTree.second.get<std::string>("msg_type");
  if (msg_type == "geometry_msgs::TransformStamped") {
    subsExternalPoseMeas_.push_back(node_.subscribe(
        subTree.second.get<std::string>("ros_topic"), 10,
        &ExternalReferenceTrackingModuleRos::externalPoseMeasTFStampedCallback,
        this, ros::TransportHints().tcpNoDelay()));
  } else if (msg_type == "nav_msgs::Odometry") {
    subsExternalPoseMeas_.push_back(node_.subscribe(
        subTree.second.get<std::string>("ros_topic"), 10,
        &ExternalReferenceTrackingModuleRos::externalPoseMeasOdomCallback, this,
        ros::TransportHints().tcpNoDelay()));
  } else if (msg_type == "OdomWithState") {
    subsExternalPoseMeas_.push_back(
        node_.subscribe(subTree.second.get<std::string>("ros_topic"), 10,
                        &ExternalReferenceTrackingModuleRos::
                            externalPoseMeasOdomWithStateCallback,
                        this, ros::TransportHints().tcpNoDelay()));
  } else {
    std::cerr << " unknown message type: " << msg_type << std::endl;
  }
}

void ExternalReferenceTrackingModuleRos::externalPoseMeasTFStampedCallback(
    const geometry_msgs::TransformStampedPtr& msg) {
  externalPoseMeasCallback(msg);
}

void ExternalReferenceTrackingModuleRos::externalPoseMeasOdomCallback(
    const nav_msgs::OdometryPtr& msg) {
  externalPoseMeasCallback(msg);
}

void ExternalReferenceTrackingModuleRos::externalPoseMeasOdomWithStateCallback(
    const confusion::OdomWithStatePtr& msg) {
  externalPoseMeasCallback(msg);
}

// todo Can we set the callbacks using the templated function directly?
template <typename MsgType>
void ExternalReferenceTrackingModuleRos::externalPoseMeasCallback(
    const MsgType& msg) {
  confusion::Pose<double> T_wa_ba_meas = confusion::getPoseFromMsg(msg);
  addMeasurement(msg->header.stamp.toSec(), msg->header.frame_id,
                 msg->child_frame_id, T_wa_ba_meas);
}

void ExternalReferenceTrackingModuleRos::publishFrameOffsetTfs() {
  ros::Time t_tf = ros::Time::now();
  std::lock_guard<std::mutex> lg(parameterQueryMtx_);
  for (auto& externalReferenceFrame : referenceFrameOffsetsCopy_) {
    tfBroadcaster_.sendTransform(
        tf::StampedTransform(getTfMsg(externalReferenceFrame.second), t_tf,
                             "world", externalReferenceFrame.first));
  }
  for (auto& sensorFrame : sensorFrameOffsetsCopy_) {
    std::string sensorFrameName;
    try {
      std::shared_ptr<confusion::ErtMeasConfig> poseMeasConfigPtr =
          sensorPoseMeasConfigs_.at(sensorFrame.first);
      sensorFrameName = poseMeasConfigPtr->sensorFrameNameForTf_;
    } catch (...) {
      std::cerr << "ERROR: Sensor frame with unknown name " << sensorFrame.first
                << " found in "
                   "ExternalReferenceTrackingModule::publishFrameOffsetTfs???"
                << std::endl;
      continue;
    }
    tfBroadcaster_.sendTransform(tf::StampedTransform(
        getTfMsg(sensorFrame.second), t_tf, "body", sensorFrameName));
  }
}

void ExternalReferenceTrackingModuleRos::
    processPoseMeasBeforeAssigningToStateDerived(
        const std::shared_ptr<ErtMeasBase>& ertMeasPtr, const double& t,
        const Pose<double>& T_w_body) {
  // Publish a tf just to see the raw measurement
  tfBroadcaster_.sendTransform(
      tf::StampedTransform(getTfMsg(ertMeasPtr->getMeasuredPose()),
                           ros::Time::now(), ertMeasPtr->referenceFrameName(),
                           ertMeasPtr->sensorFrameName() + "_meas"));
}

void ExternalReferenceTrackingModuleRos::resetSensorCallback(
    const std_msgs::StringPtr& msg) {
  resetSensor(msg->data);
}

}  // namespace confusion
