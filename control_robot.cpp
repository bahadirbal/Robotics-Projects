/************************
Name, Surname  : BAHADIR BAL
Student Number : 040090590
Assignment 1
Robotics - BLG456E
*************************/


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>					// This expresses velocity in free space broken into its linear and angular parts.
#include <sensor_msgs/LaserScan.h>					// Include the sensor_msgs/LaserScan message that publish to wire
#include <cmath>									// For some mathematical operations

class Robot_Node{                               	// Robot Class
public:												
	geometry_msgs::Twist robot_velocity;			// This expresses velocity of robot with angular and linear
	ros::Publisher pub_geometry;					// With Publisher definition to publish messages of robot movement  
	ros::Subscriber sub_scan;						// Subscriber to get informatin of laser scan 

	Robot_Node(){ 									// Constructor of Robot_Node Class
		robot_velocity.linear.x = 0.0;				// Robot's linear x coordinat of velocity is defined default zero when object is created
		robot_velocity.angular.z = 0.0;				// Robot's angular z coordinat of velocity is defined default zero when object is created
	}

	~Robot_Node() { } 								// Deconstructor of Robot_Node Class

	void movement_func(){							// To move front of robot, directly straight
        robot_velocity.linear.x = 1.0;				// for this x coordinat should be 1.0
        robot_velocity.angular.z = 0.0;				// angular z coordinat should be 0.0 
        pub_geometry.publish(robot_velocity);		// and message send by publisher over the ROS
	}

	void stop_func(){								// To stop, when execute, robot will be stopped
		robot_velocity.linear.x = 0.0;				// for this x coordinat should be 0.0
		robot_velocity.angular.z = 0.0;				// angular z coordinat should be 0.0
		pub_geometry.publish(robot_velocity);		// and message send by publisher to stop Robot for that moment
	}

	void turn_right_func(){							// To move right of robot, every communication time, Robot turns to right 1 unit
        robot_velocity.linear.x = 0.0;				// for this x coordinat should be 0.0
        robot_velocity.angular.z = (-1.0);			// angular z coordinat should be -1.0
        pub_geometry.publish(robot_velocity);		// and message send by publisher to turn right Robot for that moment
	}

	void turn_left_func(){							// To move left of robot, every communication time, Robot turns to left 1 unit
        robot_velocity.linear.x = 0.0;				// for this x coordinat should be 0.0
        robot_velocity.angular.z = 1.0;				// angular z coordinat should be 1.0
        pub_geometry.publish(robot_velocity);		// and message send by publisher to turn left Robot for that moment
	}
};

Robot_Node robot;									// define a object belongs to Robot_Node Class

void control(const sensor_msgs::LaserScan::ConstPtr& laser_scan){

	float range_min = 4.0;							// range_min hold minimum value in ranges array, firstly defined with maximum value
	float left_side = 0.0;							// left_side will hold sum of values in ranges between -135 degree and 0 degree
	float right_side = 0.0;							// right_side will hold sum of values in ranges between 0 and 135 degree

	for(unsigned int i = 0 ; i < laser_scan->ranges.size() ; i++ ){			// For loop turns size of ranges time
		if(laser_scan->ranges[i] < range_min )								// to find minumum value in ranges array
			range_min = laser_scan->ranges[i];

		if(i < laser_scan->ranges.size()/2)									// summing of left side laser nodes
			left_side += laser_scan->ranges[i];				
		else																// summing of right side laser nodes
			right_side += laser_scan->ranges[i];
	}

	if(range_min <= 0.5){							// if range_min is less than 0.5 which is point before most closest point to obstacle
		robot.stop_func();							// if yes, robot will stop and start to think which side it should go
		
		if(left_side >= right_side)					// if left side's sum is bigger than the other one, means that turn right to avoid closer obstacles
			robot.turn_right_func();
		else
			robot.turn_left_func();					// if right side's sum is bigger than the other one, means that turn left
	}
	else
		robot.movement_func();						// Otherwise, move to straight
	
	ROS_INFO(" Distance to obstacle(minimum) : %f \n", range_min);  // to show closest point to robot
	}

int main(int argc, char** argv)										// Main function
{

	ros::init(argc,argv,"robot");									// Name of cpp file is given to ros init()
	ros::NodeHandle n_;												// Define Nodehandle 
	
	robot.pub_geometry = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);  // publishing movement of robot on ROS from /cmd_vel topic
	robot.sub_scan = n_.subscribe("/base_scan/scan", 1 , control);			// subscribing info of laser scan from base_scan of Robot
	ros::spin();															// Execute when messages arrive

	return 0;
}				


// Compile & Run section
// Ros should be launched and then this code will run.
//    - roslaunch erratic_description erratic_laser_wg_world.launch
//    - rosrun assignment1_040090590 robot
