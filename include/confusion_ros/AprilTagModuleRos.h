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

#ifndef INCLUDE_CONFUSION_EXAMPLES_APRILTAGMODULEROS_H_
#define INCLUDE_CONFUSION_EXAMPLES_APRILTAGMODULEROS_H_

#include "confusion/modules/apriltag/AprilTagModule.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <std_msgs/Float64MultiArray.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/MarkerArray.h>
#include <confusion_ros/TagArray.h>
#include <tf/transform_broadcaster.h>

#include "confusion_ros/ros_conversions.h"

namespace confusion {

class AprilTagModuleRos : public AprilTagModule {
 public:
  AprilTagModuleRos(ros::NodeHandle &node,
                    ConFusor* conFusorPtr,
                    std::string dataFilePath,
                    std::string configFileName,
                    int tagMeasIndex_,
                    bool* newMeasReceivedFlag = nullptr);

  void camCalCallback(const sensor_msgs::CameraInfo::ConstPtr &msg);

  void tagArrayCallback(const confusion_ros::TagArray::ConstPtr &msg);

  void imageMsgCallback(const sensor_msgs::ImageConstPtr &msg);

  void publishTagMarkers();

private:

  ros::NodeHandle &node_;
  image_transport::ImageTransport imageTransport_;
  image_transport::Subscriber subImage_;
  ros::Subscriber subTagArray_;
  ros::Subscriber subCameraCalibration_;

  std::string cameraTopic_;
  std::string tagArrayTopic_;
  image_transport::Publisher imagePub_;
  image_transport::Publisher tagImagePub_;
  ros::Publisher pubTagMarkers_;
  ros::Publisher pubTagArray_;


  tf::TransformBroadcaster tfBroadcaster_;
};

} //namespace confusion

#endif /* INCLUDE_CONFUSION_EXAMPLES_APRILTAGMODULEROS_H_ */
