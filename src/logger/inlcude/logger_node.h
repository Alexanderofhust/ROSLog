#ifndef LOGGER_NODE_H
#define LOGGER_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <map>
#include <string>

class LoggerNode {
public:
    LoggerNode();  // 构造函数

private:
    // 回调函数，处理订阅的消息
    void callback(const std_msgs::String::ConstPtr& msg, const std::string& topic);

    // 定时器回调函数，记录日志
    void timerCallback(const ros::TimerEvent& event);

    // 成员变量
    std::vector<std::string> topics_;  // 订阅的话题列表
    std::map<std::string, std::string> topic_data_;  // 存储话题数据
    std::map<std::string, ros::Subscriber> subscribers_;  // 订阅器
    ros::Timer timer_;  // 定时器
    std::string log_comment_;  // 日志评论
};

#endif  // LOGGER_NODE_H