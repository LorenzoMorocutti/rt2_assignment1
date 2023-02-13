/**
* \file position_service.cpp
* \brief Node that generates a random
         position (x,y, \theta)
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
* ServiceServer:<BR>
*   /position_server (rt2_assignment1::RandomPosition)
*
* Description:
*
* This node has the scope to replies to a request
* for a random pose (x,y,\theta) between intervals.

*/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
* This function generates a random number 
* given an interval.
* \param M (double): lower bound
* \param N (double): upper bound
*
* \retval randMToN (double): random number
*                            between M and N.
*/
double randMToN(double M, double N)
{     return M + (rand() / ( RAND_MAX / (N-M) ) ) ; }

/**
* This function is the service callback 
* generating a random (x,y,\theta) pose
*
* \param req (rt2_assignment::RandomPosition::Request &):
*   This is the service request, that contains the (x,y) range.
* \param res (rt2_assignment1::RandomPosition::Response &):
*   This is the service response, that contains (x,y,\theta) pose.
*
* \retval myrandom (&req,&res): True
*/
bool myrandom (rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res){
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
* This function is the main. 
* It initializes the ros node and the server /position_server.
*
* \retval main : 0
*/
int main(int argc, char **argv)
{
   ros::init(argc, argv, "random_position_server");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/position_server", myrandom);
   ros::spin();

   return 0;
}
