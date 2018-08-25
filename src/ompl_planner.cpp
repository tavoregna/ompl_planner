#define raggio 0.19
#define OMPL_time 5.0

#include <pluginlib/class_list_macros.h>
#include "ompl_planner.h"
#include <stdio.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/config.h>
#include <iostream>
#include <math.h>
#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include "ros/ros.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(ompl_planner::OMPLPlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

namespace ob = ompl::base;
namespace oc = ompl::control;

int width;
int height;
double resolution;
geometry_msgs::Pose origin;
unsigned char * mapp;

double startX;
double startY;
double startYaw;

double offsetX;
double offsetY;

int pathLength;
double * pathX;
double * pathY;

bool solvedOnce;

unsigned char * getCostMap(int * wid,int * hei,double * res,geometry_msgs::Pose * origin)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");

    nav_msgs::GetMap srv;

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service static_map");
        return 0;
    }

    nav_msgs::OccupancyGrid mapp=srv.response.map;
    unsigned char * toRet;

    unsigned int width=mapp.info.width;
    unsigned int height=mapp.info.height;
    double resolution=mapp.info.resolution;
    geometry_msgs::Pose pose=mapp.info.origin;
    toRet=(unsigned char *)malloc(width*height);

    for(int i=0;i<height*width;i++)
    {

        toRet[i]=mapp.data[i];
    }

    *wid=width;
    *hei=height;
    *res=resolution;
    *origin=pose;
    return toRet;
}

void toEulerAngle(geometry_msgs::Quaternion q, double* roll, double* pitch, double* yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    *roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        *pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    *yaw = atan2(siny, cosy);
}

void getRobotPose()
{
    geometry_msgs::PoseWithCovarianceStamped msg;

    msg=*(ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose"));

    geometry_msgs::Point position=msg.pose.pose.position;
    geometry_msgs::Quaternion orientation=msg.pose.pose.orientation;

    printf("position: \n");
    printf("x: %f \n",position.x);
    printf("y: %f \n",position.y);
    printf("z: %f \n",position.z);
    printf("orientation: \n");
    printf("QUATERNION: \n");
    printf("x: %f \n",orientation.x);
    printf("y: %f \n",orientation.y);
    printf("z: %f \n",orientation.z);
    printf("w: %f \n",orientation.w);
    printf("ROLL,PITCH,YAW: \n");

    double roll,pitch,yaw;
    toEulerAngle(orientation, &roll, &pitch, &yaw);
    printf("roll: %f \n",roll);
    printf("pitch: %f \n",pitch);
    printf("yaw: %f \n",yaw);



    double yaw2   =  asin(2*orientation.x*orientation.y + 2*orientation.z*orientation.w);

    printf("yaw2: %f \n",yaw2);

    startX=position.x;
    startY=position.y;
    startYaw=yaw;
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    double x=(*pos).values[0];
    double y=(*pos).values[1];
    double yaw=(*rot).value;

    double dimCella=resolution;


    if(x<=-offsetX+raggio || y<=-offsetY+raggio || x>=offsetX-raggio || y>=offsetY-raggio)
        return false;

    x=x+offsetX;
    y=y+offsetY;

    double minX=x-raggio;
    double maxX=x+raggio;
    double minY=y-raggio;
    double maxY=y+raggio;

    

    int indexMinX=(int)floor(minX/dimCella);
    int indexMaxX=(int)floor(maxX/dimCella);
    int indexMinY=(int)floor(minY/dimCella);
    int indexMaxY=(int)floor(maxY/dimCella);


    for(int i=indexMinX;i<=indexMaxX;i++)
    {
        for(int j=indexMinY;j<=indexMaxY;j++)
        {
            if(mapp[i+j*width]!=0)
            {
                return false;
            }
        }
    }
    return true;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    result->as<ob::SE2StateSpace::StateType>()->setXY(
                pos[0] + ctrl[0] * duration * cos(rot),
            pos[1] + ctrl[0] * duration * sin(rot));
    result->as<ob::SE2StateSpace::StateType>()->setYaw(
                rot    + ctrl[1] * duration);
}

ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
{
    // we can perform any additional setup / configuration of a sampler here,
    // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

void plann()
{

    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    /*bounds.setLow(0,-offsetX);
    bounds.setHigh(0,offsetX);
    bounds.setLow(1,-offsetY);
    bounds.setHigh(1,offsetY);*/
    bounds.setLow(0,0.25);
    bounds.setHigh(0,1.2);
    bounds.setLow(1,-2.35);
    bounds.setHigh(1,2.75);

    space->setBounds(bounds);

    space->setLongestValidSegmentFraction(0.001);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0,-0.3);
    cbounds.setHigh(0,0.3);
    cbounds.setLow(1,-0.3);
    cbounds.setHigh(1,0.3);

    cspace->setBounds(cbounds);

    // construct an instance of  space information from this control space
    auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

    // set state validity checking for this space
    si->setStateValidityChecker(
                [&si](const ob::State *state) { return isStateValid(si.get(), state); });
    
    si->setStateValidityCheckingResolution(0.001);

    // set the state propagation routine
    si->setStatePropagator(propagate);

    si->setValidStateSamplerAllocator(allocOBValidStateSampler);

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(startX);
    start->setY(startY);
    //start->setYaw(startYaw);

    //start->setX(0.48);
    //start->setY(-1.64);
    start->setYaw(3.14/2.0);

    // create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(start);
    /*(*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.71;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = 2.0;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;*/
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.65;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = 1.18;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;



    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.001);

    pdef->setOptimizationObjective(getPathLengthObjective(si));

    // create a planner for the defined space

    //auto planner(std::make_shared<ompl::geometric::pSBL>(si));
    //auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));
    //auto planner(std::make_shared<ompl::geometric::RRTstar>(si));
    //auto planner(std::make_shared<ompl::geometric::BKPIECE1>(si));
    //auto planner(std::make_shared<oc::EST>(si));
    //auto planner(std::make_shared<oc::KPIECE1>(si));
    //auto planner(std::make_shared<ompl::geometric::SPARStwo>(si));
    auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));


    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(OMPL_time);

    ob::PathPtr pathh;
pathLength=-1;

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        pathh = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        pathh->print(std::cout);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
        return;
    }

    solvedOnce=true;

    ompl::geometric::PathGeometric path=*(pathh->as<ompl::geometric::PathGeometric>());
    pathLength=path.getStateCount();
    pathX=(double *)malloc(pathLength*sizeof(double));
    pathY=(double *)malloc(pathLength*sizeof(double));

    for(int i=0;i<path.getStateCount();i++)
    {
        const auto *se2state = (path.getState(i))->as<ob::SE2StateSpace::StateType>();

        const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
        const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

        double x=(*pos).values[0];
        double y=(*pos).values[1];

	pathX[i]=x;
        pathY[i]=y;

        printf("%d: X: %d, Y: %d\n",i+1,(int)floor((x+offsetX)/resolution),(int)floor((y+offsetY)/resolution));
        mapp[((int)floor((x+offsetX)/resolution))+((int)floor((y+offsetY)/resolution))*width]=3;


    }

    for(int j=0;j<height;j++)
    {
        for(int i=0;i<width;i++)
        {
            char v=mapp[i+j*width];

            if(v==-1)
                printf("\x1b[30m%-d\x1b[0m",0);
            else if(v==0)
                printf("\x1b[32m%-d\x1b[0m",1);
            else if(v==3)
                printf("\x1b[46m\x1b[36m%-s\x1b[0m","@");
            else
                printf("\x1b[33m%-d\x1b[0m",2);

        }
        printf("\n");

    }
}


 //Default Constructor
 namespace ompl_planner {

 OMPLPlanner::OMPLPlanner (){

 }

 OMPLPlanner::OMPLPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }


 void OMPLPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
	ROS_INFO("INIZIALIZZAZIONE");
	offsetX=-costmap_ros->getCostmap()->getOriginX();
        offsetY=-costmap_ros->getCostmap()->getOriginY();
        resolution=costmap_ros->getCostmap()->getResolution();
        mapp=costmap_ros->getCostmap()->getCharMap();
	width=costmap_ros->getCostmap()->getSizeInCellsX();
	height=costmap_ros->getCostmap()->getSizeInCellsY();
        
	solvedOnce=false;

	ROS_INFO("1: offsetX: %f\noffsetY: %f\nresolution: %f\nwidth: %d\nheight: %d",offsetX,offsetY,resolution,width,height);
 }

 bool OMPLPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

if(solvedOnce==true)
return true;


startX=start.pose.position.x;
startY=start.pose.position.y;
startYaw=tf::getYaw(start.pose.orientation);

/*printf("\n");
    for(int j=0;j<height;j++)
    {
        for(int i=0;i<width;i++)
        {
            char v=mapp[i+j*width];

            if(v==-1)
                printf("\x1b[30m%-d\x1b[0m",0);
            else if(v==0)
                printf("\x1b[32m%-d\x1b[0m",1);
            else if(v==3)
                printf("\x1b[46m\x1b[36m%-s\x1b[0m","@");
            else
                printf("\x1b[33m%-d\x1b[0m",2);

        }
        printf("\n");

    }*/
ROS_INFO("1: offsetX: %f\noffsetY: %f\nresolution: %f\nwidth: %d\nheight: %d",offsetX,offsetY,resolution,width,height);
mapp=getCostMap(&width,&height,&resolution,&origin);
    offsetX=-origin.position.x;
    offsetY=-origin.position.y;
//getRobotPose();
ROS_INFO("2: offsetX: %f\noffsetY: %f\nresolution: %f\nwidth: %d\nheight: %d",offsetX,offsetY,resolution,width,height);







    do{
        plann();
    }while (pathLength==-1 && solvedOnce==false);

if(pathLength!=-1)
{
    plan.push_back(start);
    for(int i=1;i<pathLength;i++)
    {
        geometry_msgs::PoseStamped new_goal = goal;
     tf::Quaternion goal_quat = tf::createQuaternionFromYaw(3.14/2.0);
	
      new_goal.pose.position.x = pathX[i];
      new_goal.pose.position.y = pathY[i];

      new_goal.pose.orientation.x = goal_quat.x();
      new_goal.pose.orientation.y = goal_quat.y();
      new_goal.pose.orientation.z = goal_quat.z();
      new_goal.pose.orientation.w = goal_quat.w();

   plan.push_back(new_goal);
    }
}
	//plan.push_back(goal);
  return true;
 }
 };

