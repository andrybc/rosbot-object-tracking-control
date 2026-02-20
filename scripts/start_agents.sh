#!/usr/bin/env bash
set -e

ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-20}

echo "[info] Starting micro-ROS agent (base) on UDP 8090..."
sudo docker run -d --rm --net=host --name microros_agent_8090 \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  microros/micro-ros-agent:jazzy udp4 --port 8090 -v4

echo "[info] Starting micro-ROS agent (camera) on UDP 9999..."
sudo docker run -d --rm --net=host --name microros_agent_9999 \
  -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
  microros/micro-ros-agent:jazzy udp4 --port 9999 -v4

echo "[ok] Agents running."
echo "Logs:"
echo "  sudo docker logs -f microros_agent_8090"
echo "  sudo docker logs -f microros_agent_9999"