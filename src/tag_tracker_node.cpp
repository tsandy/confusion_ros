//
// Created by tim on 10.10.18.
//

#include "confusion_example/ImuState.h"
#include "confusion_ros/tag_tracker_ros.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  ros::init(argc, argv, "TagTrackerRos");
  ros::NodeHandle nh;

  confusion::TagTrackerRos<ImuState> tagTracker(nh);

  ros::spin();

  return 0;
}
