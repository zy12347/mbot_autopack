#!/bin/bash

# 获取所有ROS 2节点的进程ID
NODE_PIDS=$(ps -ef | grep -i 'ros2 run' | grep -v grep | awk '{print $2}')

# 检查是否有ROS 2节点在运行
if [ -z "$NODE_PIDS" ]; then
    echo "没有检测到运行中的ROS 2节点"
    exit 0
fi

# 显示并终止所有ROS 2节点
echo "正在终止以下ROS 2节点进程："
echo $NODE_PIDS

for pid in $NODE_PIDS; do
    kill $pid
    echo "已发送终止信号到进程 $pid"
done

# 等待片刻让节点有时间停止
sleep 2

# 检查是否还有剩余节点
REMAINING_PIDS=$(ps -ef | grep -i 'ros2 run' | grep -v grep | awk '{print $2}')
if [ -z "$REMAINING_PIDS" ]; then
    echo "所有ROS 2节点已成功终止"
else
    echo "警告：仍有节点未终止，尝试强制终止..."
    for pid in $REMAINING_PIDS; do
        kill -9 $pid
        echo "已强制终止进程 $pid"
    done
fi