#include "agv_util/lifecycle_node.hpp"

namespace agv_util
{

LifecycleNode::LifecycleNode(
    const std::string & node_name,
    const std::string & node_namespace,
    const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, node_namespace, options)
{
}

LifecycleNode::~LifecycleNode()
{
    destroyBond();
}

void LifecycleNode::createBond()
{
    if (!bond_) {
        bond_ = std::make_unique<bond::Bond>(
            std::string("bond"),
            this->get_name() + std::string("_bond"),
            shared_from_this());
        bond_->setHeartbeatPeriod(0.10);
        bond_->setHeartbeatTimeout(3.0);
        bond_->start();
    }
}

void LifecycleNode::destroyBond()
{
    if (bond_) {
        bond_->breakBond();
        bond_.reset();
    }
}

}