//
// Created by tim on 14.10.19.
//

#ifndef CONFUSOR_ERTUTILITIES_H
#define CONFUSOR_ERTUTILITIES_H

#include "confusion/utilities/Pose.h"
#include "confusion_ros/OdomWithState.h"

namespace confusion {

typedef boost::shared_ptr<confusion_ros::OdomWithState> OdomWithStatePtr;

inline Pose<double> getPoseFromMsg(const OdomWithStatePtr &msg) {
  return Pose<double>(msg->pose.pose.position.x,
                      msg->pose.pose.position.y,
                      msg->pose.pose.position.z,
                      msg->pose.pose.orientation.w,
                      msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z);
}

} // namespace confusion

#endif //CONFUSOR_ERTUTILITIES_H
