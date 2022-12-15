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
  explicit NodeSub(
    const std::string & node_name_prefix,
    uint32_t node_index,
    const std::string & topic_name_prefix,
    uint32_t subscriber_count_per_node,
    uint32_t topic_count_per_node,
    bool ignore_node_index,
    bool ignore_topic_index)
  : rclcpp::Node(node_name_prefix+std::to_string(node_index))
  {
    auto dummy = [](std_msgs::msg::String::ConstSharedPtr) {};

    uint32_t topic_count_per_group = 0;
    if (topic_count_per_node != 0) {
      topic_count_per_group = (subscriber_count_per_node + (topic_count_per_node - 1)) / topic_count_per_node;
      if (topic_count_per_node > subscriber_count_per_node) {
        RCLCPP_WARN(this->get_logger(), "topic count per node [%d] is greater than subscriber count per node [%d], some topic will be ignored",
          topic_count_per_node, subscriber_count_per_node);
      }
    }

    for (uint32_t i = 0; i < subscriber_count_per_node; i++) {
      std::string topic_name;
      std::string node_index_str = ignore_node_index ? "" : "_" + std::to_string(node_index);
      uint32_t topic_index;
      if (topic_count_per_group != 0) {
        topic_index = i / topic_count_per_group;
      } else {
        topic_index = i;
      }
      std::string topic_index_str = ignore_topic_index ? "" : "_" + std::to_string(topic_index);
      topic_name = topic_name_prefix + node_index_str + topic_index_str;

      auto sub = create_subscription<std_msgs::msg::String>(topic_name, 10, dummy);
      subs_.emplace_back(sub);
    }

    RCLCPP_INFO(this->get_logger(), "Create %u subscriptions --- Done !", subscriber_count_per_node);

    auto callback = [this, subscriber_count_per_node](){
      for (auto & sub: subs_) {
        if (sub->get_publisher_count() < 1) {
          return;
        }
      }
      RCLCPP_INFO(this->get_logger(), "%s: each subscription has one publisher at least!", this->get_name());
      this->timer_->cancel();
      current_number.fetch_add(subscriber_count_per_node, std::memory_order_relaxed);
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
  std::cout << "Usage: " << prog_name
    << " -n node_count -s subscriber_count_per_node [-t topic_count_per_node] [-o topic_name_prefix] [-c ignore_node_index]" << std::endl;
}


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  char * cli_option = nullptr;
  uint32_t node_count = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-n");
  if (nullptr != cli_option) {
    node_count = stoi(std::string(cli_option));
  } else {
    usage(argv[0]);
    return EXIT_FAILURE;
  }

  uint32_t subscriber_count_per_node = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-s");
  if (nullptr != cli_option) {
    subscriber_count_per_node = stoi(std::string(cli_option));
  } else {
    usage(argv[0]);
    return EXIT_FAILURE;
  }

  uint32_t topic_count_per_node = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic_count_per_node = stoi(std::string(cli_option));
  }

  std::string topic_name_prefix = "topic_name";
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-o");
  if (nullptr != cli_option) {
    topic_name_prefix = cli_option;
  }

  std::string node_name_prefix = "node_sub_";
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-a");
  if (nullptr != cli_option) {
    node_name_prefix = cli_option;
  }

  uint32_t ignore_node_index = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-c");
  if (nullptr != cli_option) {
    ignore_node_index = stoi(std::string(cli_option));
  }

  uint32_t ignore_topic_index = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-i");
  if (nullptr != cli_option) {
    ignore_topic_index = stoi(std::string(cli_option));
  }

  // Set target
  target_number = node_count * subscriber_count_per_node;

  rclcpp::executors::SingleThreadedExecutor exe;
  std::vector<std::shared_ptr<NodeSub>> nodes;
  for (uint32_t i = 0; i < node_count; i++) {
    auto node = std::make_shared<NodeSub>(
      node_name_prefix, i, topic_name_prefix, subscriber_count_per_node,
      topic_count_per_node, ignore_node_index, ignore_topic_index);
    nodes.emplace_back(node);
    exe.add_node(node);
  }
  exe.spin();
  nodes.clear();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
