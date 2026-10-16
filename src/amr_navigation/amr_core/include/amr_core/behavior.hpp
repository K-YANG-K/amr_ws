#pragma once

#include "rclcpp/rclcpp.hpp"
#include "amr_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace amr_core{
class Behavior{
public:
    using Ptr = std::shared_ptr<Behavior>;
    virtual ~Behavior(){}

    virtual void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
     const std::string & name, std::shared_ptr<tf2_ros::Buffer> tf) = 0;

    virtual void cleanup() = 0;

    virtual void activate() = 0;

    virtual void deactivate() = 0;
    };
} 

