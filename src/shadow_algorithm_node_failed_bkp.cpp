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
geometry_msgs::Twist cmdVel;


	

//double A12, w12, alpha, l12, r12, theta1, theta2, h1, h2, k1, k2, min_vel1=0, min_vel2=0 ;
double *v;

double constrainAngle(double x);
void spawn_my_turtles(ros::NodeHandle& nh, int nTurtle, turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp);
void currPoseCallback(const turtlesim::Pose::ConstPtr& msg, int id);
//void currPoseCallback1(const turtlesim::Pose::ConstPtr& msg);
   
	   
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "defaultnode");
	ros::NodeHandle nh;
	
	turtlesim::Pose TPose;
	turtlesim::Spawn::Request req;
	turtlesim::Spawn::Response resp;
	
	int nTurtle;
	float pub_rate;
	double max_vel,safe_dia;
	ros::param::get("pubrate",	pub_rate);
	ros::param::get("safedia",	safe_dia);
	ros::param::get("nTurtle", 	nTurtle);
	ros::param::get("maxvel",	max_vel);
	ros::Rate my_rate(pub_rate);
	v = new double [nTurtle];
	cout << "pub_rate:" << pub_rate<<endl;
	cout << "safe_dia:" << safe_dia<<endl;
	cout << "nTurtle:" << nTurtle<<endl;
	cout << "max_vel:" << max_vel<<endl;
	
	//TPose = new turtlesim::Pose [nTurtle];
	
	//turtlesim::Pose *TPose;
	//TPose = new turtlesim::Pose [nTurtle];
	
	spawn_my_turtles(nh,nTurtle,req, resp);	//spawn both the turtles and initiate their pose and check if the spawnings were successful
	//cout << "This is hari sending test message" <<endl;
	//sleep(5);
	//req.name="my_turtle";
	//cout << "This is hari sending test message again" <<endl;
	//sleep(5);
	cout<<"req.name:"<<req.name<<endl;
	
	//for(int k=0; k < nTurtle; ++k)	{
		
		ros::Subscriber curr_pose_sub1 = nh.subscribe<turtlesim::Pose>("/"+req.name+"/pose", 2, boost::bind(currPoseCallback, _1,0));	// subscriber objects to get current pose of each turtle
		ros::Publisher turtle1_vel_pub = nh.advertise<geometry_msgs::Twist>("/"+req.name+"/cmd_vel", 2);	//publisher objects to publish the command velocity
		v[0]=max_vel;
		cout<<"vel:" << v[0]<<endl;
		cmdVel.linear.x=max_vel;
		turtle1_vel_pub.publish(cmdVel);
		
		my_rate.sleep();
	//}
		
	ros::spinOnce();	//this will trigger the subscriber callbacks which will fetch the turtles' current pose and vel requried for next line
	sleep(5);
	cout<<"spinning complete "<<endl;
	
	sleep(1000);

   	return 0;
}	


void spawn_my_turtles(ros::NodeHandle& nh, int nTurtle, turtlesim::Spawn::Request& req, turtlesim::Spawn::Response& resp) {
	//spawn all turtles
	ros::service::waitForService("spawn");
	ros::ServiceClient spawnTurtle = nh.serviceClient<turtlesim::Spawn>("spawn");
	
	for (int l=0; l < nTurtle; ++l) {
		cout << "(from spawn turtle)\t: " << l << endl;
	}

	//req1.name	= "myturtle1";
	ros::param::get("~T1Name",		req.name);
	ros::param::get("~T1X",		req.x);
	ros::param::get("~T1Y",		req.y);
	ros::param::get("~T1Theta",	req.theta);
	req.theta=req.theta * M_PI/180;

	//ros::param::get("~T2Name",		req2.name);
	//ros::param::get("~T2X",		req2.x);
	//ros::param::get("~T2Y",		req2.y);
	//ros::param::get("~T2Theta",	req2.theta);
	//req2.theta=req2.theta * M_PI/180;
	
	bool success1=spawnTurtle.call(req,resp);
	//bool success2=spawnTurtle.call(req2,resp2);
	//if(success1 && success2) { ROS_INFO_STREAM ("Spawned turtles"); }
	if(success1 ) { ROS_INFO_STREAM ("Spawned turtles"); }
	else { ROS_INFO_STREAM ("Error, unable to spawn turtles"); }
}

void currPoseCallback(const turtlesim::Pose::ConstPtr& msg, int id )
{
	//turtlesim::Pose TPose;
	//TPose.x 		= msg->x;
	//TPose.y 		= msg->y;
	//TPose.theta 	= constrainAngle(msg->theta);
	ROS_INFO_STREAM ("Identifier (from callback)\t: " << id );
	return;
}
double constrainAngle(double x){		//Normalize to [-180,180)
    x = fmod(x+M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}