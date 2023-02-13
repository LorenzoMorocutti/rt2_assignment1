/**
* \file state_machine.cpp
* \brief Node that gives new goals
         for the robot
* \author Lorenzo Morocutti
* \version 1.0
* \date 1/02/2023
*
* \details
* 
* Publishes to:<BR>
*   None
* 
* Subscribes to:<BR>
*   None
* 
* ServiceClient:<BR>
*   /position_server (rt2_assignment1::RandomPosition)
*
* ServiceServer:<BR>
*   /user_interface (rt2_assignment1::Coomand)
*
* Description:
*
* This node communicates with the action server for the
* go_to_point behavior, giving a new goal each time a 'start'
* request is received from the user_interface.
* It also manages the stop behavior, sotpping a running goal
* when the request is a 'stop'.
*
*/

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include "rt2_assignment1/MovingAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

bool start = false;

/**
* This function is the service callback 
* that sets the start/stop robot state.
*
* \param req (rt2_assignment1::Command::Request &):
*   Service request, containing the command (string).
* \param res (rt2_assignment1::Command::Response &):
*   Service response, the value of the 'start' state (bool).
*
* \retval user_interface() : true.
*/ 
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res){
    if (req.command == "start")
    {
        std::cout << "\nThe robot is departed" << std::endl;
    	start = true;
    }
    else 
    {
    	start = false;
        std::cout << "\nThe robot has stopped " << std::endl;
    }
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_machine");
    ros::NodeHandle n;
    ros::NodeHandle n1;
    ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
    ros::ServiceClient client_rp = n1.serviceClient<rt2_assignment1::RandomPosition>("/position_server");

    //initialize the client that uses the action
    actionlib::SimpleActionClient<rt2_assignment1::MovingAction> act_client("/go_to_point");
   
    rt2_assignment1::RandomPosition rp;
    rp.request.x_max = 5.0;
    rp.request.x_min = -5.0;
    rp.request.y_max = 5.0;
    rp.request.y_min = -5.0;
    rt2_assignment1::Position p;
    
    bool robot_is_moving = false;

    while(ros::ok())
    {
        ros::spinOnce();
   	
        if(start == true)
        {
            if(robot_is_moving == false)
            {
                client_rp.call(rp);
                rt2_assignment1::MovingGoal goal;
   		        goal.target_pose.header.frame_id = "base_link";
                goal.target_pose.header.stamp = ros::Time::now();
                goal.target_pose.pose.position.x = rp.response.x;
   		        goal.target_pose.pose.position.y = rp.response.y;
   		        goal.target_pose.pose.position.z = rp.response.theta;
   		        std::cout << "\nGoing to the position: x= " << rp.response.x << " y= " << rp.response.y << " theta = " << rp.response.theta << std::endl;
   		        act_client.waitForServer();
                act_client.sendGoal(goal);

   		        robot_is_moving = true;
            }
            else
            {
                if(act_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    std::cout << "\nThe robot has arrived " << std::endl;
                    robot_is_moving = false;
                }
            }
        }
        else
        {
            if (robot_is_moving)
            {
                act_client.cancelAllGoals();
                std::cout << "\nThe robot has stopped " << std::endl;
                robot_is_moving = false;
            }
        }   	    
    }
    
    return 0;
}
