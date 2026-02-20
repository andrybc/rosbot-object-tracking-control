#!/usr/bin/env bash
set -e

sudo docker stop microros_agent_8090 2>/dev/null || true
sudo docker stop microros_agent_9999 2>/dev/null || true
echo "[ok] Agents stopped."