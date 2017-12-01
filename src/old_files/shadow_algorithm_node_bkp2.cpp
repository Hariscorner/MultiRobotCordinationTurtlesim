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
double safe_dia, A12, w12, alpha, l12, r12, theta1, theta2, v1, v2, h1, h2, k1, k2, max_vel1, max_vel2, min_vel1=0, min_vel2=0 ;

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
	
	float pub_rate;
	ros::param::get("pubrate",	pub_rate);
	ros::param::get("safedia",	safe_dia);
	ros::Rate my_rate(pub_rate);
	
	spawn_my_turtles(nh);	//spawn both the turtles and initiate their pose and check if the spawnings were successful
	
	ros::Subscriber curr_pose_sub1 = nh.subscribe("/"+req1.name+"/pose", 2, currPoseCallback1);	// subscriber objects to get current pose of each turtle
	ros::Subscriber curr_pose_sub2 = nh.subscribe("/"+req2.name+"/pose", 2, currPoseCallback2);
	
	ros::Publisher turtle1_vel_pub = nh.advertise<geometry_msgs::Twist>("/"+req1.name+"/cmd_vel", 2);	//publisher objects to publish the command velocity
	ros::Publisher turtle2_vel_pub = nh.advertise<geometry_msgs::Twist>("/"+req2.name+"/cmd_vel", 2);

	ros::param::get("maxvel1",	max_vel1);
	ros::param::get("maxvel2",	max_vel2);
	
	v1=max_vel1;	//set the initial velocity of the robots to max
	v2=max_vel2;
	/*
	cmdVel1.linear.x=max_vel1;
	cmdVel2.linear.x=max_vel2;
	printf("going to Publish now...\n"); 
	turtle1_vel_pub.publish(cmdVel1);	
	turtle2_vel_pub.publish(cmdVel2); 
*/	
	my_rate.sleep();
	my_rate.sleep();
//cout<< "LDBLMAX\t: " <<  LDBL_MAX <<endl;
	int i=1;
	//while(ros::ok() && nh.ok() && i<100) {
		
		ros::spinOnce();	//this will trigger the subscriber callbacks which will fetch the turtles' current pose and vel requried for next line
		cout<<"spinning complete "<<endl;
		
		IloEnv   env;				//create environment handle which also creates the implementation object internally
		IloModel model(env);		//create modelling object to define optimisation models with our enviroment env
		IloNumVarArray var(env);	//create modelling variables
		IloRangeArray con(env);		//create range objects for defining constraints
		
		evalcoeffs();				//calculates the coefficients of the constraint eqns etc
		optimizeme(model,var,con);	//the real optimisation happens here
	
	while(ros::ok() && nh.ok() && i<100) {
		A12=hypot(T2Pose.x-T1Pose.x,T2Pose.y-T1Pose.y);
		cout << endl<< "A12 :" << A12  << "\t Safe_dia: " << safe_dia << "\t Difference: " << A12 - safe_dia << endl;
		
		turtle1_vel_pub.publish(cmdVel1);	
		turtle2_vel_pub.publish(cmdVel2);
		
		my_rate.sleep();
		++i;
		ros::spinOnce();
	}
		env.end();
	//}

   	return 0;
}	

static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c)
{
	
	IloEnv env = model.getEnv();		//environment handle

	x.add(IloNumVar(env, min_vel1-v1, max_vel1-v1, ILOFLOAT));	//define upper and lower bounds for variables and the variable type (continous(float) or discrete (int/bool)
	x.add(IloNumVar(env, min_vel2-v2, max_vel2-v2, ILOFLOAT));
	x.add(IloNumVar(env, 0, 1, ILOBOOL));		//boolean variables for MILP
	x.add(IloNumVar(env, 0, 1, ILOBOOL));
	x.add(IloNumVar(env, 0, 1, ILOBOOL));
	x.add(IloNumVar(env, 0, 1, ILOBOOL));

	x[0].setName("q1");		//renaming the variables
	x[1].setName("q2");
	x[2].setName("f1");
	x[3].setName("f2");
	x[4].setName("f3");
	x[5].setName("f4");
	
	
	model.add(IloMinimize(env, - x[0] - x[1]));		//objective function for minimisation (Note: done in single step instead of two) 

	c.add( -cos(theta1)*x[0] 	+ cos(theta2)*x[1] 	- x[2]*FLT_MAX <=  v1*cos(theta1) 	- v2*cos(theta2));				//defining constraints
	c.add( h1*x[0] 				- h2*x[1] 		 	- x[2]*FLT_MAX <= -v1*h1			+ v2*h2);
	c.add( cos(theta1)*x[0] 	- cos(theta2)*x[1] 	- x[3]*FLT_MAX <= -v1*cos(theta1) 	+ v2*cos(theta2));
	c.add( -h1*x[0] 			+ h2*x[1] 		 	- x[3]*FLT_MAX <=  v1*h1 			- v2*h2);

	c.add( -cos(theta1)*x[0] 	+ cos(theta2)*x[1] 	- x[4]*FLT_MAX <=  v1*cos(theta1) 	- v2*cos(theta2));				//defining constraints
	c.add( -k1*x[0] 			+ k2*x[1] 		 	- x[4]*FLT_MAX <=  v1*k1			- v2*k2);
	c.add( cos(theta1)*x[0] 	- cos(theta2)*x[1] 	- x[5]*FLT_MAX <= -v1*cos(theta1) 	+ v2*cos(theta2));
	c.add( k1*x[0] 				- k2*x[1] 		 	- x[5]*FLT_MAX <= -v1*k1 			+ v2*k2);

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
		//cplex.setOut(env.getNullStream()); 	//discard the output from next step(solving) by dumping it to null stream rather than to the screen
		
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
		
		cout<< "q1\t:" << vals[0] << "\tq2\t: " << vals[1] << endl;
		cmdVel1.linear.x=v1+vals[0];
		cmdVel2.linear.x=v2+vals[1];
		
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
	cout << endl<< "A12\t: " << A12  << "\tSafe_dia\t: " << safe_dia << "\tDifference\t: " << A12 - safe_dia << endl;
	if(A12 - safe_dia < 0) {
		cout << "Safety disc overlap identified!!!" <<endl;
		sleep(1500);
		exit (0);
	}
	else if(safe_dia/A12 > 1) {
		cout << "It's too late to avoid collision...Sine Error...exiting" <<endl;
		sleep(1500);
		exit (0);
	}
	
	alpha=asin(safe_dia/A12);
	l12=constrainAngle(w12+alpha);	//convert to the range [-180,180) if it goes out of bound
	r12=constrainAngle(w12-alpha);
	cout << "alpha\t: " << alpha <<"\tw12\t: " << w12 << "\tl12\t: " << l12 << "\tr12\t: " << r12 << endl;
	
	theta1=T1Pose.theta;
	theta2=T2Pose.theta;
	//v1=T1Pose.linear_velocity;
	//v2=T2Pose.linear_velocity;
	
	cout << "tan(l12)\t: " << tan(l12) << "\ttan(r12)\t: " << tan(r12) << endl;
	h1=tan(l12)*cos(theta1) - sin(theta1);
	h2=tan(l12)*cos(theta2) - sin(theta2);
	k1=tan(r12)*cos(theta1) - sin(theta1);
	k2=tan(r12)*cos(theta2) - sin(theta2);
	
	cout << "h1\t: " << h1  << "\th2\t: " << h2 << "\tk1\t: " << k1 << "\tk2\t: " << k2 << endl;
}
void currPoseCallback1(const turtlesim::Pose::ConstPtr& msg)
{
	T1Pose.x 		= msg->x;
	T1Pose.y 		= msg->y;
	T1Pose.theta 	= constrainAngle(msg->theta);
	//cout << "set Pose T1" <<endl;
	return;
}
void currPoseCallback2(const turtlesim::Pose::ConstPtr& msg)
{
	T2Pose.x 		= msg->x;
	T2Pose.y 		= msg->y;
	T2Pose.theta 	= constrainAngle(msg->theta);
	//cout << "set Pose T2" <<endl;
	return;
}
double constrainAngle(double x){		//Normalize to [-180,180)
    x = fmod(x+M_PI,2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}