#include <string>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <robotinfo_msgs/RobotInfo.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>

//show_received_msgs_from_topic
class CVUIROSSubscriber {
public:
  CVUIROSSubscriber();

  ros::Subscriber sub_;
  robotinfo_msgs::RobotInfo data;
  std::string topic_name;
  void msgCallback(const robotinfo_msgs::RobotInfo::ConstPtr &msg);
  const std::string WINDOW_NAME = "CVUI ROS SIMPLE SUBSCRIBER";
};

//cmd_vel
class CVUIROSCmdVelPublisher {
public:
  CVUIROSCmdVelPublisher();

  ros::Publisher twist_pub_;
  geometry_msgs::Twist twist_msg;
  std::string twist_topic_name;
  float linear_velocity_step = 0.1;
  float angular_velocity_step = 0.1;
  const std::string WINDOW_NAME = "CVUI ROS TELEOP";

  void updateTwistMsg(float velocity_step);
  void publishTwist();
};

//odom
class CVUIROSOdomSubscriber {
public:
  CVUIROSOdomSubscriber();

  ros::Subscriber sub_;
  nav_msgs::Odometry data;
  std::string topic_name;
  void msgCallback(const nav_msgs::Odometry::ConstPtr &msg);
  const std::string WINDOW_NAME = "CVUI ROS SIMPLE SUBSCRIBER";
};

//TriggerMsgServiceClient
class CVUIROSTriggerMsgServiceClient {
public:
  CVUIROSTriggerMsgServiceClient(const std::string &srv_name);

  ros::NodeHandle nh;
  ros::ServiceClient service_client;
  // Create a service request
  std_srvs::Trigger srv_req;
  std::string service_name;
  std::string last_service_call_msg;
  int service_call_counter = 0;
  const std::string WINDOW_NAME = "CVUI ROS SIMPLE SERVICE CLIENT";
};

void createShowMsgsFromTopicWindow(cv::Mat& frame, int& y, CVUIROSSubscriber& subscriber) {
  cvui::window(frame, 40, 20, 500, 100, "Topic: " + subscriber.topic_name);
  y+=15;
  cvui::printf(frame, 45, y += 15, 0.4, 0xffffff, "info received: %s", subscriber.data.data_field_01.c_str());
  cvui::printf(frame, 45, y += 15, 0.4, 0xffffff, "info received: %s", subscriber.data.data_field_02.c_str());
  cvui::printf(frame, 45, y += 15, 0.4, 0xffffff, "info received: %s", subscriber.data.data_field_03.c_str());
  cvui::printf(frame, 45, y += 15, 0.4, 0xffffff, "info received: %s", subscriber.data.data_field_04.c_str());
}

void createCmdVelWindow(cv::Mat& frame, int& y, CVUIROSCmdVelPublisher& publisher) {
  cvui::window(frame, 40, y+=30, 500, 150, "Command Velocity");
//   y += 130;
    if (cvui::button(frame, 200, y+=30, " Forward ")) {
        publisher.updateTwistMsg(publisher.linear_velocity_step);
        publisher.publishTwist();
    }
    if (cvui::button(frame, 200, y+=40, " Stop ")) {
        publisher.twist_msg.linear.x = 0.0;
        publisher.twist_msg.angular.z = 0.0;
        publisher.publishTwist();
    }
    if (cvui::button(frame, 40, y, " Left ")) {
        publisher.twist_msg.angular.z = publisher.angular_velocity_step;
        publisher.publishTwist();
    }
    if (cvui::button(frame, 400, y, " Right ")) {
        publisher.twist_msg.angular.z = -publisher.angular_velocity_step;
        publisher.publishTwist();
    }
    if (cvui::button(frame, 200, y+=40, " Backward ")) {
        publisher.twist_msg.linear.x = -publisher.linear_velocity_step;
        publisher.publishTwist();
    }

    cvui::window(frame, 40, y+=50, 300, 50, "Linear:");
    cvui::printf(frame, 45, y+=25, 0.4, 0xff0000, "%.02f m/s", publisher.twist_msg.linear.x);

    cvui::window(frame, 300, y-=25, 240, 50, "Angular:");
    cvui::printf(frame, 305, y+=25, 0.4, 0xff0000, "%.02f rad/s", publisher.twist_msg.angular.z);
}

void createOdomWindow(cv::Mat& frame, int& y, CVUIROSOdomSubscriber& subscriber) {
  cvui::printf(frame, 40, y+=30, 0.5, 0xffffff, "Estimated robot position based on odometry");
  cvui::window(frame, 40, y+=20, 300, 80, "X");
  cvui::printf(frame, 45, y+=25, 2, 0xff0000, "%0.2f", subscriber.data.pose.pose.position.x);
  cvui::window(frame, 200, y-=25, 300, 80, "Y");
  cvui::printf(frame, 205, y+=25, 2, 0xff0000, "%0.2f", subscriber.data.pose.pose.position.y);
  cvui::window(frame, 400, y-=25, 140, 80, "Z");
  cvui::printf(frame, 405, y+=25, 2, 0xff0000, "%0.2f", subscriber.data.pose.pose.position.z);
}

void createTriggerMsgServiceClientWindow(cv::Mat& frame, int& y, CVUIROSTriggerMsgServiceClient& srv_client){
  cvui::printf(frame, 40, y+=65, 0.5, 0xffffff, "Distance Travelled");
  cvui::window(frame, 150, y+=20, 390, 100, "Distance in meters: ");
  if (cvui::button(frame, 40, y, 100, 100, "Call")) {
    // Send the request and wait for a response
    if (srv_client.service_client.call(srv_client.srv_req)) {
      // Print the response message and return true
      ROS_DEBUG("Response message: %s", srv_client.srv_req.response.message.c_str());
      // set latest service call status
      srv_client.last_service_call_msg = srv_client.srv_req.response.message;
      srv_client.service_call_counter++;
    } 
    else {
      srv_client.last_service_call_msg = "Service call failed!";
      srv_client.service_call_counter = 0;
    }
  }
  if (not srv_client.last_service_call_msg.empty()) {
    cvui::printf(frame, 155, y+=30, 1.5, 0xffffff, "%s", srv_client.last_service_call_msg.c_str());
  }
}

void createParentWindow(cv::Mat& frame, CVUIROSSubscriber& subscriber, CVUIROSCmdVelPublisher& cmd_vel_pub, CVUIROSOdomSubscriber& odom_sub, CVUIROSTriggerMsgServiceClient& srv_client) {
  frame = cv::Scalar(49, 52, 49);
  int y = 20;

  createShowMsgsFromTopicWindow(frame, y, subscriber);
  createCmdVelWindow(frame, y, cmd_vel_pub);
  createOdomWindow(frame, y, odom_sub);
  createTriggerMsgServiceClientWindow(frame, y, srv_client);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robot_gui_node");
  ros::NodeHandle nh;
  CVUIROSSubscriber subscriber;
  CVUIROSCmdVelPublisher cmd_vel_publisher;
  CVUIROSOdomSubscriber odometry_subscriber;
  std::string trigger_msg_srv_name="/get_distance";
  CVUIROSTriggerMsgServiceClient trigger_msg_srv_client(trigger_msg_srv_name);
  cv::Mat frame = cv::Mat(800, 600, CV_8UC3);

  cv::namedWindow("Robot GUI");
  cvui::init("Robot GUI");

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    createParentWindow(frame, subscriber, cmd_vel_publisher, odometry_subscriber, trigger_msg_srv_client);

    cvui::update();
    cv::imshow("Robot GUI", frame);

    if (cv::waitKey(20) == 27) {
      break;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

//cmd_vel implementation
CVUIROSCmdVelPublisher::CVUIROSCmdVelPublisher()
{
  ros::NodeHandle nh;
  twist_topic_name = "cmd_vel";
  twist_pub_ = nh.advertise<geometry_msgs::Twist>(twist_topic_name, 10);
}
void CVUIROSCmdVelPublisher::updateTwistMsg(float velocity_step) {
  twist_msg.linear.x += velocity_step;
}

void CVUIROSCmdVelPublisher::publishTwist() {
  twist_pub_.publish(twist_msg);
}

//odom implementation
CVUIROSOdomSubscriber::CVUIROSOdomSubscriber() {
  ros::NodeHandle nh;
  topic_name = "odom";
  sub_ = nh.subscribe<nav_msgs::Odometry>(
      topic_name, 2, &CVUIROSOdomSubscriber::msgCallback, this);
}

void CVUIROSOdomSubscriber::msgCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  data = *msg;
}

//TriggerMsgServiceClient implementation
CVUIROSTriggerMsgServiceClient::CVUIROSTriggerMsgServiceClient(
    const std::string &srv_name) {
  // Initialize ROS node
  ros::NodeHandle nh;
  // Create a service client that sends requests of type std_srvs/Trigger
  service_client = nh.serviceClient<std_srvs::Trigger>(srv_name);
  service_name = srv_name;
}

//show_received_msgs_from_topic implementation
CVUIROSSubscriber::CVUIROSSubscriber() {
  // Initialize ROS node
  ros::NodeHandle nh;
  topic_name = "robot_info";
  sub_ = nh.subscribe<robotinfo_msgs::RobotInfo>(topic_name, 10, &CVUIROSSubscriber::msgCallback, this);
}

void CVUIROSSubscriber::msgCallback(const robotinfo_msgs::RobotInfo::ConstPtr &msg) {
  data = *msg;
  ROS_DEBUG("info received: %s", data.data_field_01.c_str());
  ROS_DEBUG("info received: %s", data.data_field_02.c_str());
  ROS_DEBUG("info received: %s", data.data_field_03.c_str());
  ROS_DEBUG("info received: %s", data.data_field_04.c_str());
}
