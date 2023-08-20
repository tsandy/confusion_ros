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

#include "confusion_ros/apriltag_module_ros.h"

namespace confusion {

AprilTagModuleRos::AprilTagModuleRos(
    ros::NodeHandle& node, ConFusor* conFusorPtr, std::string dataFilePath,
    std::string configFileName, int tagMeasIndex, bool* newMeasReceivedFlag)
    : AprilTagModule(conFusorPtr, dataFilePath, configFileName, tagMeasIndex,
                     newMeasReceivedFlag),
      node_(node),
      imageTransport_(node) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(configFileName, pt);

  pubTagMarkers_ =
      node_.advertise<visualization_msgs::MarkerArray>("/tag_markers", 10);
  tagImagePub_ = imageTransport_.advertise("/detector_image", 1);
  pubTagArray_ = node_.advertise<confusion_ros::TagArray>("/tags_detected", 10);

  // Start listening for the camera info message
  subCameraCalibration_ =
      node_.subscribe(pt.get<std::string>("camera_calibration_topic"), 10,
                      &AprilTagModuleRos::camCalCallback, this);
  cameraTopic_ = pt.get<std::string>("camera_topic");
  tagArrayTopic_ = pt.get<std::string>("tag_array_topic");

  std::cout << "[AprilTagModuleRos] Initialization complete. Waiting for the "
               "camera calibration."
            << std::endl;
}

void AprilTagModuleRos::camCalCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg) {
  subCameraCalibration_.shutdown();

  // Set the projection matrix
  Eigen::Matrix<double, 3, 4> projMat;
  projMat.setZero();
  projMat(0, 0) = msg->P[0];
  projMat(0, 2) = msg->P[2];
  projMat(1, 1) = msg->P[5];
  projMat(1, 2) = msg->P[6];
  projMat(2, 2) = 1.0;
  setCameraCalibration(projMat);

  // Start listening for camera measurements
  subImage_ = imageTransport_.subscribe(
      cameraTopic_, 2, &AprilTagModuleRos::imageMsgCallback, this);
  subTagArray_ = node_.subscribe<confusion_ros::TagArray>(
      tagArrayTopic_, 2, &AprilTagModuleRos::tagArrayCallback, this,
      ros::TransportHints().tcpNoDelay());

  // Clear any tag markers in rviz before starting
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/world";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::DELETEALL;
  markerArray.markers.push_back(marker);
  pubTagMarkers_.publish(markerArray);
}

void AprilTagModuleRos::tagArrayCallback(
    const confusion_ros::TagArray::ConstPtr& msg) {
  if (!msg->tags.empty()) {
    std::vector<TagDetection> tagDetections;
    for (const auto& tagMsg : msg->tags) {
      TagDetection td;
      td.id = (int)(tagMsg.id);
      for (int i = 0; i < 4; ++i) {
        td.corners[i][0] = tagMsg.corners[i].x;
        td.corners[i][1] = tagMsg.corners[i].y;
      }

      tagDetections.push_back(td);
    }
    processTagDetections(msg->header.stamp.toSec(), tagDetections);
  }
}

void AprilTagModuleRos::imageMsgCallback(
    const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("AprilTagModuleRos::imgaeCallback -- cv_bridge exception: %s",
              e.what());
    return;
  }

  imageCallback(msg->header.stamp.toSec(), cv_ptr->image);
}

void AprilTagModuleRos::publishTagMarkers() {
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "/world";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::MODIFY;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  int i = 0;
  for (auto& externalReferenceFrame : referenceFrameOffsets_) {
    marker.id = i;
    marker.pose = getMsg(*externalReferenceFrame.second);
    markerArray.markers.push_back(marker);
    ++i;
  }
  pubTagMarkers_.publish(markerArray);

  // Publish T_c_i tf
  std::shared_ptr<confusion::Pose<double>> T_c_i_ptr;
  try {
    T_c_i_ptr = sensorFrameOffsets_["cam"];
    tfBroadcaster_.sendTransform(confusion::getTfStampedMsg(
        T_c_i_ptr->inverse(), aprilTagParameters_.imuFrameName,
        aprilTagParameters_.cameraFrameName));
  } catch (const std::out_of_range& oor) {
    std::cerr << "ERROR: T_c_i is not initialized in "
                 "AprilTagModuleRos::publishTagMarkers!"
              << std::endl;
    return;
  }
}

}  // namespace confusion
