/*
 *=======================================================
 * SIMPLE FLOCKING ALGORITHM
 *=======================================================
 * 
 * By: Shibin Hamza Achambat, Andrea Yanez and Vafa Mammadova
 * 
 * Description
 * ---------------------------------------------------------------------------------------------------------------
 * This file implements a simulation that has a group of agents that flock together.
 * By using basic logic of the flocking behaviour implemeneted by Craig Reyonlds, i.e. 
 * by having three main features: Alignment, Cohesion and Separation, a simple flocking
 * behaviour where the agents exhibit a coordinated motion is simulated. Equipped with
 * sensors to detect neighbourhood agents and having info of their location and speed, 
 * each agent adjusts its control signal such that flocking behaviour is achieved. 
 * The control signal has weighted inputs to achieve alignment, cohesion and separation.
 * These weights can be modified to observe the behaviour of agents under different
 * conditions.
 * 
*/

//Include all the required header files

#include "neuralanimat.h"
#include "sensor.h"
#include "population.h"
#include "vector2d.h"
#include <vector>
#include "math.h"
#include <functional>
#include <fstream>
#include <iostream>


using namespace std;
using namespace BEAST;



//-------------------------------------------------------
// NUMBER OF AGENTS
// Change this variable if you want to change the number
// of agents in a simulation-	

double agentPopulation_Simple = 40;								
//-------------------------------------------------------


//-----------------------------------------------------------------------
// Agent class equipped with all necessary sensors, alignment algorithms.
// Has sensors to detect nearby agents.
// The sensor values, along with flocking parameters are directly wired
// to the control signal to left and right wheels through weights.
//-----------------------------------------------------------------------
class Agent : public Animat
{

public:
	//...........................................................
	// Constructor of the class
	// The agents are initiated at a random location and have 
	// sensors to detect angle and distance to the nearest agent.
	//...........................................................
	Agent()
    {        
	This.InitRandom = true;									
	This.Add("angle", NearestAngleSensor<Agent>());				// Nearest angle sensor to the nearest agent
	This.Add("Xdist", NearestXSensor<Agent>());					// This sensor outputs X distance to nearest agent
	This.Add("Ydist", NearestYSensor<Agent>());					// This sensor outputs Y distance to nearest agent
    }

	//..................................................................
	// Default control function overwritten for case specific algorithm
	//..................................................................
    virtual void Control()
    {
		////		SENSOR OUTPUTS STORED IN VARIABLE		////
		double xdist = This.Sensors["Xdist"]->GetOutput();
		double ydist = This.Sensors["Ydist"]->GetOutput();
		double angle_ = This.Sensors["angle"]->GetOutput();
		
		double dist = pow((pow(xdist,2)+pow(ydist,2)),0.5);			// Distance to nearest agent, calculated from sensor outputs
		double left_control, right_control;							// Variables to assign control signal to agent wheels
		
		////			agents is a vector of agents in the current world		////
		agents.clear();												// Clear the vector in each simulation	
		GetWorld().Get(agents);										// Assign the current world agent objects to the agents vector
	

		//
		// The alignment, cohesion vector stored to use in control 
		// signal formula
		//
		Vector2D alignment = This.computeAlignment(agents,scan_radius);			// Calculate Alignment vector
		Vector2D cohesion = This.computeCohesion(agents, scan_radius); 			// Calculate Cohesion vector
		
		////		COMPUTE SEPARATION		//// 
		double separation_angle;	
		double separationDistance = 0.05;

		//
		// If the distance to the nearest agent is above the separation distance, then this 
		// agent will move towards the nearest agent and is the distance is less then it 
		// will move away from the neighbour
		//
		if (dist > separationDistance) 
		{	
			separation_angle = angle_;
		} 
		else {
			
			// Below is an adjustment to get a standardized angle between -PI and +PI
			if (angle_<0) 
			{		
				separation_angle = angle_+3.14;
			}
			else {
				separation_angle = angle_-3.14;
			}
		
		}	

		//
		// Compute Cohesion and Alignment angles, derived from the 
		// alignment, cohesion vectors calculated earlier. This 
		// orientation value will be used in the formula for control
		// signal.	
		// 
		double align_angle= getAngle(alignment);
		double cohesion_angle = atan(cohesion.y / cohesion.x);

		//
		// Compute the required orientation of this animat
		// angleThis is a combination of the current orientation of the animat with
		// weighted contributions from alignment, cohesion, separation orientations
		//
		double angleThis;
		Vector2D velThis = this->GetVelocity();
		angleThis = getAngle(velThis)-align_weight*align_angle-cohesion_weight*cohesion_angle-separation_weight*separation_angle;

		//
		// Set values for right and left control
		// The control signal is calculated such that both left and right wheels 
		// will have the same default control signal defined by the def_ctrl variable
		// and when angleThis>0 the right control value will increase and when the 
		// value of angleThis<0 the left control value will increase. This enables a
		// change of orientation of the agent towards angleThis value with reference
		// to the current orientation of the agent.
		//
		double right_ctrl = def_ctrl + (angleThis>0?angleThis:0)*(def_ctrl/3.14) ;
		double left_ctrl = def_ctrl - (angleThis<0?angleThis:0)*(def_ctrl/3.14) ;
		

		// Assign the respective output to the control signal variables
		This.Controls["right"] = right_ctrl;
		This.Controls["left"] = left_ctrl;
	
    }   

	//....................................................................
	// This is called when an agent collides with any object in the World.
	//....................................................................
	virtual void OnCollision(WorldObject* obj)
	{
		Animat::OnCollision(obj);
	}

	//.............................................................
	//Function to compute alignment vector
	//.............................................................
	Vector2D computeAlignment(vector<Agent*> allAgents, double scan_radius) 
	{
		Vector2D avgVel = Vector2D(0,0);
		int count = 0;	

		//
		// Below for loop calculates the total velocity of all the agents within the 
		// scan radius from this agent
		//	
		for(auto& agent: allAgents) 
		{
			if(agent==this) 							// Do not evaluate for the current agent
			{
				continue;
			} 
			else {
				//
				// Computes total velocity of all other agents in the population
				// within the scan radius
				//
				if((this->GetLocation()-agent->GetLocation()).GetLength()<scan_radius) 
				{			
					avgVel = avgVel+agent->GetVelocity();
					count++;
				}
			}
		}
		
		if (count==0) 
		{
			return avgVel;
		} 
		else {
			avgVel = avgVel*((1.0f)/(double)count);					// Calculate average velocity
			double a = atan(avgVel.y/avgVel.x);						// Orientation of average velocity
			
			return avgVel;											// Average velocity of the neighbourhood is returned
		}

	}

	//........................................................
	// Function to compute cohesion vector
	//........................................................
	 Vector2D computeCohesion(vector<Agent*> allAgents, double scan_radius) 
	 {
		Vector2D avgPos = Vector2D(0, 0); // Initialize average position
		int count = 0;        

		//
		// Below for loop calculates the sum of position of all the 
		// agents within the scan radius. 
		//
		for(auto& agent: allAgents) 
		{
		    //
			// Do not calculate position of the current agent 
			//
			if(agent == this) 
			{
		        continue;
		    } 
			else {
		        double distance = (this->GetLocation() - agent->GetLocation()).GetLength();

		        if(distance < scan_radius) 
				{
		            avgPos += agent->GetLocation(); 					// Accumulate position for cohesion
		            count++;
		        }
		    }
		}

		if (count == 0) 
		{
		    return avgPos;
		} 
		else {
		    avgPos.x /= count; 											// Calculate average x position
		    avgPos.y /= count;											// Calculate average y position

		    Vector2D cohesion = avgPos - this->GetLocation();			// Cohesion value is the vector connecting position of current agent
		    return cohesion;											// and the average position of the neighbourhood.
		}
   	 }

	//.................................................................
	// Accessors that will later be needed to get genotypes
	//.................................................................
	double GetAlignWeight ()const 
	{
		return align_weight;					// Get alignment weight
	}

	double GetCohesionWeight ()const 
	{
		return cohesion_weight;					// Get cohesion weight
	}

	double GetSeparationWeight ()const 
	{
		return separation_weight;				// Get separation weight
	}

	double GetScanRadius ()const 
	{
		return scan_radius;						// Get Scan radius used for alignment and cohesion
	}

	double GetDefControl ()const 
	{
		return def_ctrl;						// Get speed default value
	}	

	//................................................................
	// Mutators that will later be needed to assign genotypes
	//................................................................
	void SetAlignWeight(double a) 
	{
		align_weight = a;						// Set alignment weight					
	}
	void SetCohesionWeight(double a) 
	{
		cohesion_weight = a;					// Set cohesion weight		
	}
	void SetSeparationWeight(double a) 
	{
		separation_weight = a;					// Set separation weight
	}
	void SetScanRadius(double a) 
	{
		scan_radius = a;						// Set scan radius
	}
	void SetDefControl(double a) 
	{
		def_ctrl = a;							// Set default speed
	}

	//........................................................................
	// Function to compute angle. This outputs angle in range -PI to PI	
	//........................................................................
	double getAngle(Vector2D vect_) const 
	{
		double angleThis;
		if (vect_.x>=0) 
		{	
			angleThis = atan(vect_.y/vect_.x);				// Value is ok when x>0
		} 
		else {
			if (vect_.y<0) 
			{
				angleThis = atan(vect_.y/vect_.x)-3.14;		// atan returns 0:PI/2 in this case, offset of -PI is given
			} 
			else {
				angleThis = 3.14+atan(vect_.y/vect_.x);		// atan returns 0:-PI/2 in this case, offset of PI is given
			}
		}
		return angleThis;	
	}
	
	vector<Agent*> agents;	

private:

	//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
	// SET PARAMETERS FOR THE FLOCKING
	// Change the following parameters to check the effect of each of the parameters
	// on the flocking behaviour. Each weight indicates the effect of the respective
	// orientation output on the control signal to the agent wheels. The scan radius 
	// value can be changed to increase the neighbourhood radius for the alignment
	// and cohesion logic. The def_ctrl indicates the default control signal to the 
	// wheeels. The def_ctrl value is proportional to the rpm output to the wheel.
	//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
	double align_weight = 1;					// Alignment weight
	double cohesion_weight = 0.4;				// Cohesion weight
	double separation_weight = 0.4;				// Separation weight
	double scan_radius = 200.0;					// Scan Radius for alignment and cohesion
	double def_ctrl = 0.7;						// Default speed control signal
	//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
};

//-------------------------------------------------------------------------------------------------
// Create simulation for the flocking algorithm
//-------------------------------------------------------------------------------------------------
class SimpleFlockingSimulation : public Simulation
{
	Group<Agent> theBirds;			// Group of agents

public:
	SimpleFlockingSimulation():
	theBirds(agentPopulation_Simple)
	{		
	
		This.SetTimeSteps(-1);									// Set timesteps per generation
		This.Add("Agent", theBirds);							// Add the agents/birds to the simulation

	}
};


