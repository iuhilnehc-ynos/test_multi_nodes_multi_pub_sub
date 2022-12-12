#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <rclcpp/subscription_base.hpp>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

uint32_t target_number = 0;
std::atomic_uint32_t current_number = 0;

class NodeSub : public rclcpp::Node
{
public:
  explicit NodeSub(const std::string & node_name, uint32_t sub_num) : rclcpp::Node(node_name)
  {
    std::string topic_basename = node_name;
    topic_basename.replace(4, 5, "");

    auto dummy = [](std_msgs::msg::String::ConstSharedPtr msg) {(void) msg;};

    for (uint32_t i = 0; i < sub_num; i++) {
      std::string topic_name = topic_basename + "_" + std::to_string(i);
      auto sub = create_subscription<std_msgs::msg::String>(topic_name, 10, dummy);
      subs_.emplace_back(sub);
    }

    RCLCPP_INFO(this->get_logger(), "Create %u subscriptions --- Done !", sub_num);

    auto callback = [this](){
      for (auto & sub: subs_) {
        if (sub->get_publisher_count() != 1) {
          return;
        }
      }
      RCLCPP_INFO(this->get_logger(), "%s: each subscription has one publisher!", this->get_name());
      this->timer_->cancel();
      current_number.fetch_add(10, std::memory_order_relaxed);
      if (current_number == target_number) {
        RCLCPP_INFO(this->get_logger(), "+++ All subscriptions connect publishers ! +++");
      }
    };

    timer_ = create_wall_timer(std::chrono::milliseconds(500), callback);
  }

private:
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

void usage(std::string prog_name){
  std::cout << "Usage: " << prog_name << " -n Node_Num -s Sub_Num_In_One_Node" << std::endl;
}


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  char * cli_option = nullptr;
  uint32_t node_num = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-n");
  if (nullptr != cli_option) {
    node_num = stoi(std::string(cli_option));
  } else {
    usage(argv[0]);
    return EXIT_FAILURE;
  }

  uint32_t sub_num = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-s");
  if (nullptr != cli_option) {
    sub_num = stoi(std::string(cli_option));
  } else {
    usage(argv[0]);
    return EXIT_FAILURE;
  }

  // Set target
  target_number = node_num * sub_num;

  rclcpp::executors::SingleThreadedExecutor exe;
  std::vector<std::shared_ptr<NodeSub>> nodes;
  for (uint32_t i = 0; i < node_num; i++) {
    std::string node_name = "node_sub_" + std::to_string(i);
    auto node = std::make_shared<NodeSub>(node_name, sub_num);
    nodes.emplace_back(node);
    exe.add_node(node);
  }
  exe.spin();
  nodes.clear();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
