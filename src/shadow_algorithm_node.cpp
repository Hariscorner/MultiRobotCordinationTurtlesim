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
#define NTURTLE 3

//double A12, w12, alpha, l12, r12, theta1, theta2, h1, h2, k1, k2, min_vel1=0, min_vel2=0 ;

double constrainAngle(double x);
void spawn_my_turtles(ros::NodeHandle& nh, int nTurtle, turtlesim::Spawn::Request req[], turtlesim::Spawn::Response resp[]);
void currPoseCallback(const turtlesim::Pose::ConstPtr& msg, int id, turtlesim::Pose TPose[]);
   
	   
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "defaultnode");
	ros::NodeHandle nh;
	ros::Subscriber curr_pose_sub[NTURTLE];
	ros::Publisher turtle_vel_pub[NTURTLE];
	
	int nTurtle=NTURTLE;
	double max_vel,v, safe_dia,pub_rate;
	
	geometry_msgs::Twist cmdVel[NTURTLE];
	turtlesim::Spawn::Request req[NTURTLE];
	turtlesim::Spawn::Response resp[NTURTLE];
	turtlesim::Pose TPose[3];
	
	ros::param::get("pubrate",	pub_rate);
	ros::param::get("safedia",	safe_dia);
	ros::param::get("maxvel",	max_vel);
	//ros::param::get("nTurtle",	nTurtle);
	
	ros::Rate my_rate(pub_rate);
	
	spawn_my_turtles(nh, nTurtle, req, resp);	//spawn both the turtles and initiate their pose and check if the spawnings were successful
	
	for (int m=0; m<nTurtle; ++m) {
		//ros::Subscriber curr_pose_sub = nh.subscribe<turtlesim::Pose>("/"+req[m].name+"/pose", 2, boost::bind(currPoseCallback, _1,m, TPose));	// subscriber objects to get current pose of each turtle
		//ros::Publisher turtle_vel_pub = nh.advertise<geometry_msgs::Twist>("/"+req[m].name+"/cmd_vel", 2);	//publisher objects to publish the command velocity
		
		curr_pose_sub[m] = nh.subscribe<turtlesim::Pose>("/"+req[m].name+"/pose", 2, boost::bind(currPoseCallback, _1,m, TPose));	// subscriber objects to get current pose of each turtle
		turtle_vel_pub[m] = nh.advertise<geometry_msgs::Twist>("/"+req[m].name+"/cmd_vel", 2);	//publisher objects to publish the command velocity
		//v=max_vel;	//set the initial velocity of the robots to max
		cmdVel[m].linear.x=max_vel;
	}
	
	
	
	
	//cout<<"going to Publish now: \t"<<cmdVel.linear.x<<endl; 
	int i=1;
	while(ros::ok() && nh.ok() && i<8){
		for (int h=0; h<nTurtle; ++h) {
			turtle_vel_pub[h].publish(cmdVel[h]);
		}
		my_rate.sleep();
		ros::spinOnce();
		my_rate.sleep();
		++i;
	}
//	cout<<"TPose[0].x" << TPose[0].x<<endl;
//	cout<<"TPose[0].y" << TPose[0].y<<endl;
//	cout<<"TPose[0].theta" << TPose[0].theta<<endl;
//	cout<<"TPose[1].x" << TPose[1].x<<endl;
//	cout<<"TPose[1].y" << TPose[1].y<<endl;
//	cout<<"TPose[1].theta" << TPose[1].theta<<endl;
	cout<<"done!"<<endl;
	sleep(1000);
   	return 0;
}	



void spawn_my_turtles(ros::NodeHandle& nh,int nTurtle, turtlesim::Spawn::Request req[], turtlesim::Spawn::Response resp[]) {
	//spawn two turtles
	ros::service::waitForService("spawn");
	ros::ServiceClient spawnTurtle = nh.serviceClient<turtlesim::Spawn>("spawn");
	
	double x=5.5, y=5.5, p=0, r=5.5;
	
	for (int j=0; j<nTurtle; ++j) {
		req[j].name="myturtle";
		req[j].name += std::to_string(j+1);
		p=j*2*M_PI/nTurtle;
		req[j].x=x+r*cos(p);
		req[j].y=y+r*sin(p);
		req[j].theta=M_PI+p;
		//req[j].theta=req[j].theta * M_PI/180;
		
//		cout << "req[j].x : " << req[j].x <<endl;
//		cout << "req[j].y : " << req[j].y <<endl;
//		cout << "req[j].theta : " << req[j].theta <<endl;
		
		bool success=spawnTurtle.call(req[j],resp[j]);
		if(success) { ROS_INFO_STREAM ("Spawned turtle\t:" << j ); }
		else { ROS_INFO_STREAM ("Error, unable to spawn turtle"); }
	}
}

void currPoseCallback(const turtlesim::Pose::ConstPtr& msg, int id, turtlesim::Pose TPose[]) {
	TPose[id].x 		= msg->x;
	TPose[id].y 		= msg->y;
	TPose[id].theta 	= constrainAngle(msg->theta);
	//cout << "Turtle Identifier: " << id <<endl;
	//cout<<"TPose.x" << TPose[id].x<<endl;
	//cout<<"TPose.y" << TPose[id].y<<endl;
	//cout<<"TPose.theta" << TPose[id].theta<<endl;
	return;
}

double constrainAngle(double x){		//Normalize to [-180,180)
    x = fmod(x+M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}