#include <memory>
#include <iostream>
#include <chrono>
#include <functional>
#include <string>
#include "rt2_assignment1_ros2/srv/position.hpp"
#include "rt2_assignment1_ros2/srv/command.hpp"
#include "rt2_assignment1_ros2/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std;
using Comm = rt2_assignment1_ros2::srv::Command;
using Pos = rt2_assignment1_ros2::srv::Position;
using RandPos = rt2_assignment1_ros2::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

bool start = false;
bool do_once = true; 

namespace rt2_assignment1_ros2
{
    class GoalToReachClient: public rclcpp::Node
    {
        public:
            GoalToReachClient() : Node("GoalToReach")
                {
                    goto_client = this->create_client<Pos>("/go_to_point");
                    this->req = std::make_shared<Pos::Request>();
                    this->res = std::make_shared<Pos::Response>();
                }

            void GoTo_server()
            {
                auto goto_result = goto_client->async_send_request(req);
                if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), goto_result)!=rclcpp::FutureReturnCode::SUCCESS)
                {
                    RCLCPP_ERROR(this->get_logger(), "Err: service has failed to be called");
                } 
                this->res = goto_result.get();
            }
            std::shared_ptr<Pos::Request> req;
            std::shared_ptr<Pos::Response> res;
        
        private:
            rclcpp::Client<Pos>::SharedPtr goto_client;
    };

    class FSM : public rclcpp :: Node
    {
        public:
            FSM(const rclcpp::NodeOptions & options) : Node("FiniteStateMac", options)
            {
                Timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&FSM::Timer_clbk, this));
                service = this->create_service<Comm>("/user_interface", std::bind(&FSM::srv_handle, this, _1, _2, _3));
                RandPosClient = this->create_client<RandPos>("/position_server");
            }

        private:
            void srv_handle(const std::shared_ptr<rmw_request_id_t> header, 
                            const std::shared_ptr<Comm::Request> request, 
                            const std::shared_ptr<Comm::Response> response)
            {
                (void) header;
                if(request->command == "stop")
                {
                    std::cout<<"stop received"<<std::endl;
                    start = false;
                    response->ok = true;
                }
                else
                {
                    start = true;
                    response->ok = true;
                }
            }

            void Timer_clbk()
            {
                auto r_pos = std::make_shared<RandPos::Request>();
                if(do_once == false)
                {
                    do_once = true;
                }
                else if(start == false)
                {
                    
                }
                else if(start)
                {
                    r_pos->x_max = 5.0;
                    r_pos->x_min = -5.0;
                    r_pos->y_max = 5.0;
                    r_pos->y_min = -5.0;

                    using SrvFuture = rclcpp::Client<RandPos>::SharedFuture;
                    auto got_response_clbk = [this] (SrvFuture future)
                    {
                        auto goal = std::make_shared<GoalToReachClient>();
                        goal->req->x=future.get()->x;
                        goal->req->y=future.get()->y;
                        goal->req->theta=future.get()->theta;
                        std::cout << "\n Going to: x= " << goal->req->x << " y= " << goal->req->y << " theta= " << goal->req->theta << std::endl;
                        goal->GoTo_server();
                        std::cout<<"Goal has been reached"<<std::endl;
                    };

                    auto goto_result = RandPosClient->async_send_request(r_pos, got_response_clbk);
                    do_once = false; 
                }
            }

            rclcpp::Service<Comm>::SharedPtr service;
            rclcpp::TimerBase::SharedPtr Timer;
            rclcpp::Client<RandPos>::SharedPtr RandPosClient;
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1_ros2::FSM)