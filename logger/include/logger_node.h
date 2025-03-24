#ifndef LOGGER_NODE_H
#define LOGGER_NODE_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <map>
#include <string>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>

class LoggerNode {
public:
    LoggerNode();  // 构造函数

    ~LoggerNode();

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
    std::ofstream log_file_;  // 日志文件
    std::string log_path_;  // 日志文件路径

    // 获取当前时间并格式化为字符串
    std::string getCurrentTimeString() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        return ss.str();
    }
};

#endif  // LOGGER_NODE_H