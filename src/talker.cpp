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


/** @file talker.cpp
 *  @brief A publisher node that publishes a string message
 *  @copyright (c) BSD License
 *  @author Chin-Po Tsai
 */
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
// #include "String.h"
#include "beginner_tutorials/change_string.h"
#include <tf/transform_broadcaster.h>



// String s;

// / the default string for the publisher
extern std::string text = "wakanda forever ";


/**
*   @brief This function changes the content of the published message
*   @param req, resp (for request and respond)
*   @return bool
*/
bool changeString(beginner_tutorials::change_string::Request& req,
                  beginner_tutorials::change_string::Response& resp) {
  resp.out = req.in;
  
  // s.text = resp.out;
  text = resp.out;

  ROS_WARN_STREAM("Changing the output String");
  return true;
}


/**
 * This publisher demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;



  // Creating objects of class TransformBroadcaster and Transform
  tf::TransformBroadcaster br;
  tf::Transform transform;



  /// the default publisher frequency
  int frequency = 10;

  /// setting the frequency value by launch file
  if (argc == 2) {
    frequency = atoi(argv[1]);
    ROS_DEBUG_STREAM("The input argument is " << frequency);
  }
  /// Warning if the frequency is less than 0
  if (frequency < 0) {
    ROS_ERROR_STREAM("the frequency is INVALID");
    frequency = 5;
    ROS_WARN_STREAM("the frequency is forced to be 5");
  }
  /// Showing Fatal message if frequency is 0
  if (frequency == 0) {
    ROS_FATAL_STREAM("Frequency is 0");
  }

  ros::Rate loop_rate(frequency);






  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);


  auto server = n.advertiseService("change_string", changeString);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;




    // setting the origin and rotation for the transform object
    transform.setOrigin(tf::Vector3(2.0, 3.0, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, 6.28);
    transform.setRotation(q);
    // braoadcasting the transform using Transformbroadcaster
    br.sendTransform(tf::StampedTransform(transform,
                    ros::Time::now(), "world", "talk"));



    std::stringstream ss;
    

    // ss << s.text << " " << count;
    ss << text << " " << count;


    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
