#include "uwall.h"
#include <math.h>
#define PI 3.141592
#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
#define WALL_DISTANCE 1
#define MAX_SPEED 0.5
#define P 10    // Proportional constant for controller
#define D 5     // Derivative constant for controller
#define ANGLE_COEF 1    // Proportional constant for angle controller
#define DIRECTION 1 // 1 for wall on the left side of the robot (-1 for the right side).
#define PUBLISHER_TOPIC "husky_velocity_controller/cmd_vel"
#define SUBSCRIBER_TOPIC "scan"


float min_range_ = 5.0;
bool obstacle_ = false;
bool debug=true;

float linear=0.0;
float angular=0.0;

WallFollowing::WallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an)
{
  wallDistance = wallDist;
  maxSpeed = maxSp;
  direction = dir;
  p = pr;
  d = di;
  angleCoef = an;
  e = 0;
  angleMin = 0;  //angle, at which was measured the shortest distance
  pubMessage = pub;
}

WallFollowing::~WallFollowing()
{
}

//publisher

void WallFollowing::publishMessage()
{
  //preparing message
  geometry_msgs::Twist msg;

  msg.angular.z = direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2);    //PD controller

  if (distFront < wallDistance){
    msg.linear.x = 0;
  }
  else if (distFront < wallDistance * 2){
    msg.linear.x = 0.5*maxSpeed;
  }
  else if (fabs(angleMin)>1.75){
    msg.linear.x = 0.4*maxSpeed;
  } 
  else {
    msg.linear.x = maxSpeed;
  }

  //publishing message
  pubMessage.publish(msg);
}

//subscriber
void WallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int size = msg->ranges.size();

  //Variables whith index of highest and lowest value in array.
  int minIndex = 0;
  int maxIndex = 0;

  //This cycle goes through array and finds minimum
  //i=minindex; i<maxindex
  float lowestDistance = 100;
  for(int i = 0; i < msg->ranges.size(); i++)
  {
    //std:printf("range %d: %f ", i, msg->ranges[i]);

    if (msg->ranges[i]>0.5 && msg->ranges[i] < lowestDistance)
    {
      lowestDistance =  msg->ranges[i]; 
      minIndex=i;
    }
    //if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i]> 0.0){
     // minIndex = i;
    //}
  }

  //Calculation of angles from indexes and storing data to class variables.
  angleMin = (minIndex-size/2)*msg->angle_increment;
  double distMin;
  distMin = msg->ranges[minIndex];
  distFront = msg->ranges[size/2];
  
  diffE = (distMin - wallDistance) - e;
  e = distMin - wallDistance;

  if(debug==true)
  {
    ROS_INFO("minindex: %d", minIndex);
    ROS_INFO("maxindex: %d", maxIndex);

    ROS_INFO("anglemin :%f", angleMin);
    ROS_INFO("distmin: %f ",distMin);
    ROS_INFO("e: %f", e);
    ROS_INFO("diffE %f", diffE);
    ROS_INFO("distFront %f", distFront);
    ROS_INFO("wallDistance: %f", wallDistance);
  }
  
  //Invoking method for publishing message
  publishMessage();
}




int main(int argc, char **argv) 
{
  //Initializes ROS, and sets up a node
  ros::init(argc, argv, "wallFollowing");
  ros::NodeHandle nh;

  //ros::Subscriber subOdom = nh.subscribe("husky_velocity_controller/odom", 100, odomCallback);
 
  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);


  WallFollowing *WallFollow = new WallFollowing(pub, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, 1);

  ros::Subscriber subLaser = nh.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &WallFollowing::messageCallback, WallFollow);


ros::spin();

  return 0;

  /*
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
  }*/
}