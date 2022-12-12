#!/bin/bash

function usage() {
  echo "Usage: run_test.sh NUM_NODE NUM_PUB_OR_SUB"
  echo "        NUM_NODE: the number of node separately created for publisher and subscription"
  echo "  NUM_PUB_OR_SUB: the number of created publisher/subscription for each node"
  exit 0
}

if [ $# -ne 2 ];then
  usage
fi

node_num=$1
pub_sub_num=$2

ros2 run test_multi_nodes_multi_pub_sub multi_nodes_multi_pub -n ${node_num} -p ${pub_sub_num} &

sleep 3

ros2 run test_multi_nodes_multi_pub_sub multi_nodes_multi_sub -n ${node_num} -s ${pub_sub_num}