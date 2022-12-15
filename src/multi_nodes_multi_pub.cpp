#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/string.hpp"

class NodePub : public rclcpp::Node
{
public:
  explicit NodePub(
    const std::string & node_name_prefix,
    uint32_t node_index,
    const std::string & topic_name_prefix,
    uint32_t publisher_count_per_node,
    uint32_t topic_count_per_node,
    bool ignore_node_index,
    bool ignore_topic_index)
  : rclcpp::Node(node_name_prefix+std::to_string(node_index))
  {
    uint32_t topic_count_per_group = 0;
    if (topic_count_per_node != 0) {
      topic_count_per_group =
        (publisher_count_per_node + (topic_count_per_node - 1)) / topic_count_per_node;;
      if (topic_count_per_node > publisher_count_per_node) {
        RCLCPP_WARN(this->get_logger(), "topic count per node [%d] is greater than publisher count per node [%d], some topic will be ignored",
          topic_count_per_node, publisher_count_per_node);
      }
    }

    for (uint32_t i = 0; i < publisher_count_per_node; i++) {
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

      auto pub = create_publisher<std_msgs::msg::String>(topic_name, 10);
      pubs_.emplace_back(pub);
    }

    RCLCPP_INFO(this->get_logger(), "Create %u publishers --- Done !", publisher_count_per_node);

    auto callback = [this](){
      for (auto & pub: pubs_) {
        if (pub->get_subscription_count() < 1) {
          return;
        }
      }
      RCLCPP_INFO(this->get_logger(), "%s: each publisher has one subscription at least!", this->get_name());
      this->timer_->cancel();
    };

    timer_ = create_wall_timer(std::chrono::milliseconds(500), callback);
  }

private:
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> pubs_;
  rclcpp::TimerBase::SharedPtr timer_;
};

void usage(std::string prog_name){
  std::cout << "Usage: " << prog_name
    << " -n node_count -p publisher_count_per_node [-t topic_count_per_node] [-o topic_name_prefix] [-c ignore_node_index] [-i ignore_topic_index]" << std::endl;
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

  uint32_t publisher_count_per_node = 0;
  cli_option = rcutils_cli_get_option(argv, argv + argc, "-p");
  if (nullptr != cli_option) {
    publisher_count_per_node = stoi(std::string(cli_option));
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

  std::string node_name_prefix = "node_pub_";
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

  rclcpp::executors::SingleThreadedExecutor exe;
  std::vector<std::shared_ptr<NodePub>> nodes;
  for (uint32_t i = 0; i < node_count; i++) {
    auto node = std::make_shared<NodePub>(
      node_name_prefix, i, topic_name_prefix, publisher_count_per_node,
      topic_count_per_node, ignore_node_index, ignore_topic_index);
    nodes.emplace_back(node);
    exe.add_node(node);
  }
  exe.spin();

  nodes.clear();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
