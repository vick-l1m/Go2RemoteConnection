#ifndef COMMAND_TRANSFER_H
#define COMMAND_TRANSFER_H

#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"
#include "geometry_msgs/msg/twist.hpp"

static const float maxSpeed = 0.8;
static const float maxTurnSpeed = 1.2;
static const float maxTiltSpeed = 0.2; 

class CommandTransfer : public rclcpp::Node
{
public:
    CommandTransfer();

private:

    void commandCallback(geometry_msgs::msg::Twist::SharedPtr data);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commandSuber_;

    unitree_api::msg::Request reqMsg_;
    SportClient sportClient_;
};

#endif  // COMMAND_TRANSFER_H
