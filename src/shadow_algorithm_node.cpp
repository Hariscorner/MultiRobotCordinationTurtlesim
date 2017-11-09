#include <ros/ros.h>
#include <iostream>
#include <turtlesim/Spawn.h>
#include <ilcplex/ilocplex.h>
#include <chrono>	//for endl
#include <math.h> 	//for hypot
#include "turtlesim/Pose.h"		
#include <limits.h>
#include "geometry_msgs/Twist.h"

using namespace std;
geometry_msgs::Twist cmdVel1 ,cmdVel2;
turtlesim::Spawn::Request req1, req2;
turtlesim::Spawn::Response resp1, resp2;
turtlesim::Pose T1Pose, T2Pose;
double safe_dia, A12, w12, alpha, l12, r12, theta1, theta2, v1, v2, h1, h2, k1, k2;

void evalcoeffs();
double constrainAngle(double x);
void spawn_my_turtles(ros::NodeHandle nh);
void currPoseCallback1(const turtlesim::Pose::ConstPtr& msg);
void currPoseCallback2(const turtlesim::Pose::ConstPtr& msg);
void optimizeme(IloModel model, IloNumVarArray var, IloRangeArray con);
static void populatebyrow (IloModel model, IloNumVarArray var, IloRangeArray con);
   //usage (const char *progname),
   
	   
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "defaultnode");
	ros::NodeHandle nh;
	ros::Rate my_rate(1);
	ros::param::get("safedia",	safe_dia);
	
	spawn_my_turtles(nh);	//spawn both the turtles and initiate their pose and check if the spawnings were successful
	
	ros::Subscriber curr_pose_sub1 = nh.subscribe("/"+req1.name+"/pose", 5, currPoseCallback1);	// subscriber objects to get current pose of each turtle
	ros::Subscriber curr_pose_sub2 = nh.subscribe("/"+req2.name+"/pose", 5, currPoseCallback2);
	
	ros::Publisher turtle1_vel_pub = nh.advertise<geometry_msgs::Twist>("/"+req1.name+"/cmd_vel", 1000);	//publisher objects to publish the command velocity
	ros::Publisher turtle2_vel_pub = nh.advertise<geometry_msgs::Twist>("/"+req2.name+"/cmd_vel", 1000);
	
	cmdVel1.linear.x 	= 2; 	cmdVel1.angular.z 	= 0;	//initial cmd_vel for turtle
	cmdVel2.linear.x 	= 2; 	cmdVel2.angular.z 	= 0;

	printf("going to Publish now...\n"); 
	my_rate.sleep();
	my_rate.sleep();
	//publish the velocity once
	turtle1_vel_pub.publish(cmdVel1);	
	turtle2_vel_pub.publish(cmdVel2);

	int i=1;
	while(ros::ok() && nh.ok() && i<8) {
		
		ros::spinOnce();	//this will trigger the subscriber callbacks which will fetch the turtles' current pose and vel requried for next line

		IloEnv   env;				//create environment handle which also creates the implementation object internally
		IloModel model(env);		//create modelling object to define optimisation models with our enviroment env
		IloNumVarArray var(env);	//create modelling variables
		IloRangeArray con(env);		//create range objects for defining constraints
		
		evalcoeffs();				//calculates the coefficients of the constraint eqns etc
		optimizeme(model,var,con);	//the real optimisation happens here
		
		turtle1_vel_pub.publish(cmdVel1);	
		turtle2_vel_pub.publish(cmdVel2);
		
		my_rate.sleep();
		++i;
		env.end();
	}

   	return 0;
}	

static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c)
{
	IloEnv env = model.getEnv();		//environment handle

	x.add(IloNumVar(env, 0.0, 2.0, ILOFLOAT));	//define upper and lower bounds for variables and the variable type (continous(float) or discrete (int/bool)
	x.add(IloNumVar(env, 0.0, 2.0, ILOFLOAT));
	x.add(IloNumVar(env, 0, 1, ILOBOOL));		//boolean variables for MILP
	x.add(IloNumVar(env, 0, 1, ILOBOOL));
	x.add(IloNumVar(env, 0, 1, ILOBOOL));
	x.add(IloNumVar(env, 0, 1, ILOBOOL));

	x[0].setName("q1");		//renaming the variables
	x[1].setName("q2");
	x[2].setName("f1");
	x[2].setName("f2");
	x[2].setName("f3");
	x[2].setName("f4");
	
	
	model.add(IloMinimize(env, - x[0] - x[1]));		//objective function for minimisation (Note: done in single step instead of two) 

	c.add( -cos(theta1)*x[0] 	+ cos(theta2)*x[1] 	- x[2]*INT_MAX <= v1*cos(theta1) 	- v2*cos(theta2));				//defining constraints
	c.add( h1*x[0] 				- h2*x[1] 		 	- x[2]*INT_MAX <= -v1*h1			+ v2*h2);
	c.add( cos(theta1)*x[0] 	- cos(theta2)*x[1] 	- x[3]*INT_MAX <= -v1*cos(theta1) 	+ v2*cos(theta2));
	c.add( -h1*x[0] 			- h2*x[1] 		 	- x[3]*INT_MAX <= v1*h1 			- v2*h2);

	c.add( -cos(theta1)*x[0] 	+ cos(theta2)*x[1] 	- x[4]*INT_MAX <= v1*cos(theta1) 	- v2*cos(theta2));				//defining constraints
	c.add( -k1*x[0] 			- k2*x[1] 		 	- x[4]*INT_MAX <= v1*k1				- v2*k2);
	c.add( cos(theta1)*x[0] 	- cos(theta2)*x[1] 	- x[5]*INT_MAX <= -v1*cos(theta1) 	+ v2*cos(theta2));
	c.add( k1*x[0] 				- k2*x[1] 		 	- x[5]*INT_MAX <= -v1*k1 			+ v2*k2);

	c.add( x[2] + x[3] + x[4] + x[5]							   <= 3);

	c[0].setName("c1");		//renaming the constraints
	c[1].setName("c2");
	c[2].setName("c3");
	c[3].setName("c4");
	c[4].setName("c5");
	c[5].setName("c6");
	c[6].setName("c7");
	c[7].setName("c8");
	c[8].setName("c9");
	model.add(c);			//adding constraints to the model

}  // END populatebyrow

void optimizeme(IloModel model, IloNumVarArray var, IloRangeArray con) {
	IloEnv env = model.getEnv();
    try {
		populatebyrow (model, var, con);	//populate the model
		
      	IloCplex cplex(model);				//create cplex object for solving the problem (Note: two steps combined)
      	cplex.exportModel("lpex1.lp");		//write the extracted model to lplex1.lp file (optional)
		cplex.setOut(env.getNullStream()); 	//discard the output from next step(solving) by dumping it to null stream rather than to the screen
		
		// Optimize the problem and obtain solution.
		if ( !cplex.solve() ) { //this returns an ILOBOOL value (true/false) depending on whether it obtained a feasible soln or not
		 env.error() << "Failed to optimize LP" << endl;
		 throw(-1);
		}
		
		IloNumArray vals(env);	//to store the solution values of all the variables
		env.out() << "Solution status = " << cplex.getStatus() << endl;		//to get more detailed status of the soln
		env.out() << "Solution value  = " << cplex.getObjValue() << endl;	//query the objective function value that resulted from optimisation
		
		cplex.getValues(vals, var);		//solution values for all variables will be stored to vals
		env.out() << "Values        = " << vals << endl;
		
		cmdVel1.linear.x=vals[0];
		cmdVel2.linear.x=vals[1];
		
	}
   	catch (IloException& e) {
      	cerr << "Concert exception caught: " << e << endl;
  	}
   	catch (...) {
      	cerr << "Unknown exception caught" << endl;
   	}//end of try catch
}

void spawn_my_turtles(ros::NodeHandle nh) {
	//spawn two turtles
	ros::service::waitForService("spawn");
	ros::ServiceClient spawnTurtle = nh.serviceClient<turtlesim::Spawn>("spawn");

	//req1.name	= "myturtle1";
	ros::param::get("~T1Name",		req1.name);
	ros::param::get("~T1X",		req1.x);
	ros::param::get("~T1Y",		req1.y);
	ros::param::get("~T1Theta",	req1.theta);
	req1.theta=req1.theta * M_PI/180;

	ros::param::get("~T2Name",		req2.name);
	ros::param::get("~T2X",		req2.x);
	ros::param::get("~T2Y",		req2.y);
	ros::param::get("~T2Theta",	req2.theta);
	req2.theta=req2.theta * M_PI/180;
	
	bool success1=spawnTurtle.call(req1,resp1);
	bool success2=spawnTurtle.call(req2,resp2);
	if(success1 && success2) { ROS_INFO_STREAM ("Spawned turtles"); }
	else { ROS_INFO_STREAM ("Error, unable to spawn turtles"); }
}

void evalcoeffs() {
	A12=hypot(T2Pose.x-T1Pose.x,T2Pose.y-T1Pose.y);
	w12=atan2(T2Pose.y-T1Pose.y,T2Pose.x-T1Pose.x);
	alpha=asin(safe_dia/A12);
	l12=w12+alpha;
	r12=w12-alpha;
	
	theta1=T1Pose.theta;
	theta2=T2Pose.theta;
	v1=T1Pose.linear_velocity;
	v2=T2Pose.linear_velocity;
	
	h1=tan(l12)*cos(theta1) - sin(theta1);
	h2=tan(l12)*cos(theta2) - sin(theta2);
	k1=tan(r12)*cos(theta1) - sin(theta1);
	k2=tan(r12)*cos(theta2) - sin(theta2);
}
void currPoseCallback1(const turtlesim::Pose::ConstPtr& msg)
{
	T1Pose.x 		= msg->x;
	T1Pose.y 		= msg->y;
	T1Pose.theta 	= constrainAngle(msg->theta);
	return;
}
void currPoseCallback2(const turtlesim::Pose::ConstPtr& msg)
{
	T2Pose.x 		= msg->x;
	T2Pose.y 		= msg->y;
	T2Pose.theta 	= constrainAngle(msg->theta);
	return;
}
double constrainAngle(double x){
    x = fmod(x+M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}