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

bool debug=true;

enum State
{
  SEARCHING = 1,
  FOLLOWING = 2
};

int state;
bool updated;
float factor= 1.0;



WallSearching::WallSearching(ros::Publisher pub, double wallDist, double maxSp)
{
  pubMessage=pub;
  wallDistance=wallDist;
  maxSpeed=maxSp;
  angle=1.0;

}

WallSearching::~WallSearching()
{
}

void WallSearching::publishMessage()
{
  geometry_msgs::Twist msg;

  
  if( distFront <= wallDistance*1.5)
  {
    msg.linear.x=0.0;
    state=2;
    updated=false;
  }
   if(distFront<wallDistance*6)
  {
    msg.linear.x=maxSpeed;
  }
  else
  {
    msg.linear.x=maxSpeed/factor;
    angle-=0.01*factor;
    factor=factor-factor/100;
    msg.angular.z=angle;
  }

  ROS_INFO("angle: %f", angle);
  pubMessage.publish(msg);

}

void WallSearching::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int size = msg->ranges.size();

  distFront= msg->ranges[size/2];
  ROS_INFO("distFront: %f", distFront);
  ROS_INFO("wallDistance2: %f", wallDistance*2);

  publishMessage();
}


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

  if (distFront < wallDistance/2){
    msg.linear.x = -1;
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

  ROS_INFO("angular= %f", msg.angular.z);
  //publishing message
  pubMessage.publish(msg);
}

//subscriber
void WallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  int size = msg->ranges.size();

  //Variables whith index of lowest value in array.
  int minIndex = 0;
  
  float lowestDistance = 100;
  for(int i = 0; i < msg->ranges.size(); i++)
  {
    
    //the 0.4 is used to filter some error measurements
    if (msg->ranges[i]>0.4 && msg->ranges[i] < lowestDistance)
    {
      lowestDistance =  msg->ranges[i]; 
      minIndex=i;
    }
    
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
    ROS_INFO("angle_increment: %f", msg->angle_increment);
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
  
  state=1;
  updated=false;
  
  ros::Publisher pub=nh.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);

  //WallFollowing *wallFollow = new WallFollowing(pub, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, ANGLE_COEF);

  //ros::Subscriber subFollow = nh.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &WallFollowing::messageCallback, wallFollow);
  WallSearching *wallSearch;
  ros::Subscriber subSearch;
  WallFollowing *wallFollow;
  ros::Subscriber subFollow;
  
  ros::Rate rate(10);

while(ros::ok()) 
  {
    ROS_INFO("state: %d", state);
    if(updated == false)
    {
      switch (state) {
      case 1:
      wallSearch = new WallSearching(pub, WALL_DISTANCE, MAX_SPEED);
      subSearch = nh.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &WallSearching::messageCallback, wallSearch);
      updated=true;
      break;

      case 2:
      
      wallFollow = new WallFollowing(pub, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, ANGLE_COEF);
      subFollow = nh.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &WallFollowing::messageCallback, wallFollow);
      delete wallSearch;
      subSearch.shutdown(); 
      updated=true;
      break;

      default:
        printf("eish\n");
      break;
      }
    }

    ros::spinOnce();
    rate.sleep();
  }

  return 0;

}