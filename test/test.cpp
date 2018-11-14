/********************************************************************
 *  MIT License

 *  Copyright <2018> <Chin-Po Tsai>

 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 *  documentation files (the "Software"), to deal in the Software without restriction, including without limitation 
 *  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, 
 *  and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

 *  The above copyright notice and this permission notice shall be included in all copies or substantial portions
 *  of the Software.

 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
 *  TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
 *  CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS 
 *  IN THE SOFTWARE.
 ********************************************************************/


/** @file test.cpp
 *  @brief Implementing the test case
 *  @copyright BSD License
 *  @author Chin-Po Tsai
 */
#include <ros/ros.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/change_string.h"

/**
*   @ brief test the existence of service
*   @ param TESTSuite testService
*   @ return none
*/
TEST(TESTSuite, testService) {
  ros::NodeHandle n;
  auto client =
      n.serviceClient<beginner_tutorials::change_string>("change_string");
  bool exists(client.waitForExistence(ros::Duration(10)));
  EXPECT_TRUE(exists);
}
