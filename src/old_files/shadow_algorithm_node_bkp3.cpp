//all variables changed from global to local and passed accordingly to relevant funcitons as pass by reference

#include <ros/ros.h>
#include <iostream>
#include <turtlesim/Spawn.h>
#include <ilcplex/ilocplex.h>
#include <chrono>	//for endl
#include <math.h> 	//for hypot
#include "turtlesim/Pose.h"		
#include <limits.h>
#include "geometry_msgs/Twist.h"
#include <boost/assign/list_of.hpp>

using namespace std;

//double A12, w12, alpha, l12, r12, theta1, theta2, h1, h2, k1, k2, min_vel1=0, min_vel2=0 ;

double constrainAngle(double x);
void spawn_my_turtles(ros::NodeHandle& nh, turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp);
void currPoseCallback(const turtlesim::Pose::ConstPtr& msg, int id, turtlesim::Pose& TPose);
   
	   
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "defaultnode");
	ros::NodeHandle nh;
	
	double max_vel,v, safe_dia;
	
	geometry_msgs::Twist cmdVel;
	turtlesim::Spawn::Request req;
	turtlesim::Spawn::Response resp;
	turtlesim::Pose TPose;
	
	float pub_rate;
	ros::param::get("pubrate",	pub_rate);
	ros::param::get("safedia",	safe_dia);
	ros::Rate my_rate(pub_rate);
	
	spawn_my_turtles(nh, req, resp);	//spawn both the turtles and initiate their pose and check if the spawnings were successful
	
	ros::Subscriber curr_pose_sub = nh.subscribe<turtlesim::Pose>("/"+req.name+"/pose", 2, boost::bind(currPoseCallback, _1,0, TPose));	// subscriber objects to get current pose of each turtle
	
	ros::Publisher turtle_vel_pub = nh.advertise<geometry_msgs::Twist>("/"+req.name+"/cmd_vel", 2);	//publisher objects to publish the command velocity

	ros::param::get("maxvel",	max_vel);
	
	v=max_vel;	//set the initial velocity of the robots to max
	
	cmdVel.linear.x=max_vel;
	cout<<"going to Publish now: \t"<<cmdVel.linear.x<<endl; 
	
	while(ros::ok() && nh.ok()){
	turtle_vel_pub.publish(cmdVel);	
	my_rate.sleep();
	ros::spinOnce();
	my_rate.sleep();
		  }
	cout<<"done!"<<endl;
	sleep(1000);
   	return 0;
}	



void spawn_my_turtles(ros::NodeHandle& nh, turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp) {
	//spawn two turtles
	ros::service::waitForService("spawn");
	ros::ServiceClient spawnTurtle = nh.serviceClient<turtlesim::Spawn>("spawn");

	//req.name	= "myturtle1";
	ros::param::get("~T1Name",		req.name);
	ros::param::get("~T1X",		req.x);
	ros::param::get("~T1Y",		req.y);
	ros::param::get("~T1Theta",	req.theta);
	req.theta=req.theta * M_PI/180;
	
	bool success=spawnTurtle.call(req,resp);
	
	if(success) { ROS_INFO_STREAM ("Spawned turtles"); }
	else { ROS_INFO_STREAM ("Error, unable to spawn turtles"); }
}

void currPoseCallback(const turtlesim::Pose::ConstPtr& msg, int id, turtlesim::Pose& TPose) {
	TPose.x 		= msg->x;
	TPose.y 		= msg->y;
	TPose.theta 	= constrainAngle(msg->theta);
	cout << "Turtle Identifier: " << id <<endl;
	return;
}

double constrainAngle(double x){		//Normalize to [-180,180)
    x = fmod(x+M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}