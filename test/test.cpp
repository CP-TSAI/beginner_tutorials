#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/change_string.h"


TEST(TESTSuite, testService) {
  ros::NodeHandle n;
  auto client = n.serviceClient<beginner_tutorials::change_string>("change_string");
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}
