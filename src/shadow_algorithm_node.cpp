#include <ros/ros.h>
#include <ilcplex/ilocplex.h>
#include <chrono>	//for endl

using namespace std;

void optimizeme();
static void populatebyrow     (IloModel model, IloNumVarArray var, IloRangeArray con);
   //usage (const char *progname),
   
	   
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "defaultnode");
	ros::NodeHandle nh;
	
	optimizeme();


   return 0;
}	

void optimizeme() {
		IloEnv   env;					//create environment handle which also creates the implementation object internally
	IloModel model(env);		//create modelling object to define optimisation models with our enviroment env
	IloNumVarArray var(env);	//create modelling variables
	IloRangeArray con(env);		//create range objects for defining constraints
	
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

   env.end();
}
static void
populatebyrow (IloModel model, IloNumVarArray x, IloRangeArray c)
{
   IloEnv env = model.getEnv();		//environment handle

   x.add(IloNumVar(env, 0.0, 40.0, ILOFLOAT));	//define upper and lower bounds for variables and the variable type (continous(float) or discrete (int/bool)
   x.add(IloNumVar(env, 0.0, 10.0, ILOINT));
   x.add(IloNumVar(env, 15.0, 23.0, ILOFLOAT));

   model.add(IloMaximize(env, x[0] + 2 * x[1] + 3 * x[2]));		//objective function for maximisation (Note: done in single step instead of two) 

   c.add( - x[0] +     x[1] + x[2] <= 20);				//defining constraints
   c.add(   x[0] - 3 * x[1] + x[2] <= 30);

   x[0].setName("x1");		//renaming the variables
   x[1].setName("x2");
   x[2].setName("x3");

   c[0].setName("c1");		//renaming the constraints
   c[1].setName("c2");
   model.add(c);			//adding constraints to the model

}  // END populatebyrow