#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h>
#include "std_msgs/String.h" 
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

float min_range_ = 5.0;
bool obstacle_ = false;
bool debug=true;

float linear=0.0;
float angular=0.0;


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{ 
  if(debug==true)
  {
    ROS_INFO("Seq: [%d]", msg->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
  }
  
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  std::vector<float> ranges(msg->ranges.begin(), msg->ranges.end());
  std::vector<float>::iterator range_it = std::max_element(ranges.begin(), ranges.end());



float average=0.0;
  for(int i=0; i<ranges.size(); i++)
  {
    average+=ranges[i];
  //::printf("%i : %f \n", i, ranges[i]);  
  }
  
  average=average/ranges.size();


  if(ranges[ranges.size()/2] < 1.0 || ranges[ranges.size()/4] < 1.0 ) obstacle_ = true;
  else obstacle_ = false;

  if(obstacle_==false)
  {
    linear=0.4;
    angular=0.0;
     ROS_INFO("no obstacle \n");
  }
  else
  {
    linear=0.0;
    angular=-0.4;
    ROS_INFO("obstacle \n");
  }

            if(debug==true){
  ROS_INFO("vlinear %lf", linear);
  ROS_INFO("vangular %lf", angular);
  ROS_INFO("average: %f", average);
  ROS_INFO("frontdistance: %f",ranges[ranges.size()/2] );
  ROS_INFO("quarterrange: %f", ranges[ranges.size()/4]);
  }
}

int main(int argc, char **argv) 
{
  //Initializes ROS, and sets up a node
  ros::init(argc, argv, "squarewall");
  ros::NodeHandle nh;

  //ros::Subscriber subOdom = nh.subscribe("husky_velocity_controller/odom", 100, odomCallback);
 
  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>("husky_velocity_controller/cmd_vel", 100);

  ros::Subscriber subLaser = nh.subscribe("scan", 10, laserCallback);

  //Sets the loop to publish at a rate of 10Hz
  ros::Rate rate(10);

 

  while(ros::ok()) 
  {
    //Declares the message to be sent
    geometry_msgs::Twist msg;
          
    msg.linear.x = linear;
    msg.angular.z = angular;

    pub.publish(msg);
     
    ros::spinOnce();

    //Delays untill it is time to send another message
    rate.sleep();
  }
}