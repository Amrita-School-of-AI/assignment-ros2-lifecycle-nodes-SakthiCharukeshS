#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class LifecycleNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit LifecycleNode(const std::string & node_name, bool intra_process_comms = false)
  : rclcpp_lifecycle::LifecycleNode(node_name,
      rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &)
  {
    pub_ = this->create_publisher<std_msgs::msg::String>("lifecycle_chatter", 10);
    timer_ = this->create_wall_timer(
      1s, std::bind(&LifecycleNode::publish_message, this));

    RCLCPP_INFO(get_logger(), "on_configure() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &)
  {
    pub_->on_activate();
    RCLCPP_INFO(get_logger(), "on_activate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &)
  {
    pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "on_deactivate() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &)
  {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on_shutdown() is called.");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  void publish_message()
  {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = "Lifecycle Hello World: " + std::to_string(count_++);

    if (pub_->is_activated()) {
      RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg->data.c_str());
      pub_->publish(std::move(msg));
    }
  }

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  unsigned int count_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  
  // FIX: Node name MUST be "lifecycle_node" for the test to find it
  std::shared_ptr<LifecycleNode> lc_node =
    std::make_shared<LifecycleNode>("lifecycle_node");

  exe.add_node(lc_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}