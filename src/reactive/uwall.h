#include <ros/ros.h>
#include <geometry_msgs/Twist.h> 
#include <stdlib.h>
#include "std_msgs/String.h" 
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"

class WallFollowing
{
public:
	WallFollowing(ros::Publisher pub, double wallDist, double maxSp, int dir, double pr, double di, double an);
	~WallFollowing();

	void publishMessage();

	void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

	double wallDistance; // Desired distance from the wall.
	double e;            // Difference between desired distance from the wall and actual distance.
	double diffE;     // Derivative element for PD controller;
	double maxSpeed;     // Maximum speed of robot.
	double p;            // k_P Constant for PD controller.
	double d;            // k_D Constant for PD controller.
	double angleCoef;    // Coefficient for P controller.
	int direction;      // 1 for wall on the right side of the robot (-1 for the left one).
	double angleMin;     // Angle, at which was measured the shortest distance.
	double distFront;    // Distance, measured by ranger in front of robot.
	ros::Publisher pubMessage;  // Object for publishing messages.
};