//
// Created by tim on 17.01.20.
//

#ifndef CONFUSION_ROS_APRILTAG_ROS_UTILS_H
#define CONFUSION_ROS_APRILTAG_ROS_UTILS_H

#include <ros/ros.h>

#include "confusion/modules/apriltag/external/april_tag_detector.hpp"
#include "confusion_ros/TagArray.h"
#include "confusion_ros/TagMsg.h"

TagDetection getTagDetection(const confusion::TagMsg& tagMsg) {
  TagDetection tdOut;
  tdOut.id = (int)(tagMsg.id);
  for (int i = 0; i < 4; ++i) {
    tdOut.corners[i][0] = tagMsg.corners[i].x;
    tdOut.corners[i][1] = tagMsg.corners[i].y;
  }
}

// Publish a tagArray message from the TagDetections
confusion::TagArray getTagArrayMessage(ros::Time t,
                                       std::vector<TagDetection> td) {
  // publish where the corners are and (if cam infos available) and estimation
  // of the tag pose wrt to the cam
  confusion::TagMsg tag;
  confusion::TagArray tags;
  tags.header.stamp = t;

  for (size_t i = 0; i < td.size(); ++i) {
    // set the tag id in the message
    tag.id = td[i].id;

    // Fill in the corner points
    for (int j = 0; j < 4; ++j) {
      tag.corners[j].x = td[i].corners[j][0];
      tag.corners[j].y = td[i].corners[j][1];
      tag.corners[j].z = 0;
    }

    tags.tags.push_back(tag);
  }

  return tags;
}

#endif  // CONFUSION_ROS_APRILTAG_ROS_UTILS_H
