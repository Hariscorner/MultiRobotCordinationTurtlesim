#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <ilcplex/ilocplex.h>
#include <chrono>	//for endl
#include <math.h> 	//for hypot
#include "turtlesim/Pose.h"		
#include <limits.h>

using namespace std;
turtlesim::Pose T1Pose, T2Pose;
float d=0.5, A12, w12, alpha, l12, r12, theta1, theta2, v1, v2, h1, h2, k1, k2;

void evalcoeffs();
void currPoseCallback1(const turtlesim::Pose::ConstPtr& msg);
void currPoseCallback2(const turtlesim::Pose::ConstPtr& msg);
void optimizeme(IloModel model, IloNumVarArray var, IloRangeArray con);
static void populatebyrow (IloModel model, IloNumVarArray var, IloRangeArray con);
   //usage (const char *progname),
   
	   
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "defaultnode");
	ros::NodeHandle nh;
	
	//spawn two turtles
	ros::service::waitForService("spawn");
	ros::ServiceClient spawnTurtle = nh.serviceClient<turtlesim::Spawn>("spawn");
	
	turtlesim::Spawn::Request req1, req2;
	turtlesim::Spawn::Response resp1, resp2;
	
	req1.name	= "myturtle1";
	req1.x 		= 4;
	req1.y		= 2;
	req1.theta	= M_PI/3;
	
	req2.name	= "myturtle2";
	req2.x 		= 2;
	req2.y		= 4;
	req2.theta	= M_PI/6;
	
	bool success1=spawnTurtle.call(req1,resp1);
	bool success2=spawnTurtle.call(req2,resp2);
	if(success1 && success2) { ROS_INFO_STREAM ("Spawned turtles"); }
	else { ROS_INFO_STREAM ("Error, unable to spawn turtles"); }
	
	// subscriber to get current pose of turtle
	ros::Subscriber curr_pose_sub1 = nh.subscribe("/myturtle1/pose", 5, currPoseCallback1);
	ros::Subscriber curr_pose_sub2 = nh.subscribe("/myturtle2/pose", 5, currPoseCallback2);

	evalcoeffs();
	
	IloEnv   env;					//create environment handle which also creates the implementation object internally
	IloModel model(env);		//create modelling object to define optimisation models with our enviroment env
	IloNumVarArray var(env);	//create modelling variables
	IloRangeArray con(env);		//create range objects for defining constraints
	
	ros::Rate my_rate(.01);
	//while(ros::ok() && nh.ok()){
		ros::spinOnce();
		optimizeme(model,var,con);
		printf("Processing...\n"); 
		my_rate.sleep();
	//}
	env.end();
   	return 0;
}	

static void populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c)
{
	IloEnv env = model.getEnv();		//environment handle

	x.add(IloNumVar(env, 0.0, 5.0, ILOFLOAT));	//define upper and lower bounds for variables and the variable type (continous(float) or discrete (int/bool)
	x.add(IloNumVar(env, 0.0, 5.0, ILOFLOAT));
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
		
      	IloCplex cplex(model);			//create cplex object for solving the problem (Note: two steps combined)
      	cplex.exportModel("lpex1.lp");	//write the extracted model to lplex1.lp file (optional)
		
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
		
	}
   	catch (IloException& e) {
      	cerr << "Concert exception caught: " << e << endl;
  	}
   	catch (...) {
      	cerr << "Unknown exception caught" << endl;
   	}//end of try catch
}

void evalcoeffs() {
	A12=hypot(T2Pose.x-T1Pose.x,T2Pose.y-T1Pose.y);
	w12=atan2(T2Pose.y-T1Pose.y,T2Pose.x-T1Pose.x);
	alpha=asin(d/A12);
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
	ROS_INFO("Received T1 pose");
	T1Pose.x 		= msg->x;
	T1Pose.y 		= msg->y;
	T1Pose.theta 	= msg->theta;
	return;
}
void currPoseCallback2(const turtlesim::Pose::ConstPtr& msg)
{
	ROS_INFO("Received T2 pose");
	T2Pose.x 		= msg->x;
	T2Pose.y 		= msg->y;
	T2Pose.theta 	= msg->theta;
	return;
}