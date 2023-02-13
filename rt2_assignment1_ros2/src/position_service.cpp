#include <inttypes.h>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rt2_assignment1_ros2/srv/random_position.hpp"

using namespace std;
using RandPos = rt2_assignment1_ros2::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1_ros2 
{
    class RP : public rclcpp :: Node
    {
        public:
            RP(const rclcpp::NodeOptions & options) : Node("position", options)
            {
                serv = this->create_service<RandPos>("/position_server", std::bind(&RP::handle_serv, this, _1, _2, _3));
            }

        private:
            double randMToN(double M, double N)
            {
                return M + (rand() / ( RAND_MAX / (N-M)));
            }

            void handle_serv(
                const std::shared_ptr<rmw_request_id_t> request_header,
                const std::shared_ptr<RandPos::Request> req,
                const std::shared_ptr<RandPos::Response> res
            ) 
            {
                (void)request_header;
                    res->x = randMToN(req->x_min, req->x_max);
                    res->y = randMToN(req->y_min, req->y_max);
                    res->theta = randMToN(-3.14, 3.14);
            }

            rclcpp::Service<RandPos>::SharedPtr serv;
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1_ros2::RP)
