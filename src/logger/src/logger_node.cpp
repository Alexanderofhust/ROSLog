#include "logger_node.h"
#include <ros/ros.h>

// 构造函数
LoggerNode::LoggerNode() {
    // 初始化节点
    ros::NodeHandle nh("~");

    // 从参数服务器加载配置
    nh.getParam("topics", topics_);
    nh.param<std::string>("log_comment", log_comment_, "Log: ");

    // 初始化订阅器
    for (const auto& topic : topics_) {
        subscribers_[topic] = nh.subscribe<std_msgs::String>(
            topic, 10, boost::bind(&LoggerNode::callback, this, _1, topic));
        topic_data_[topic] = "N/A";  // 初始化数据
    }

    // 创建定时器（1秒周期）
    timer_ = nh.createTimer(ros::Duration(1.0), &LoggerNode::timerCallback, this);
}

// 回调函数，处理订阅的消息
void LoggerNode::callback(const std_msgs::String::ConstPtr& msg, const std::string& topic) {
    topic_data_[topic] = msg->data;
}

// 定时器回调函数，记录日志
void LoggerNode::timerCallback(const ros::TimerEvent& event) {
    // 动态获取最新评论（允许实时调整）
    ros::param::get("~log_comment", log_comment_);

    // 拼接所有话题数据
    std::string log_str = log_comment_;
    for (const auto& topic : topics_) {
        log_str += topic + ": " + topic_data_[topic] + " ";
    }

    // 输出日志
    ROS_INFO("%s", log_str.c_str());
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "logger_node");
    LoggerNode node;  // 创建LoggerNode实例
    ros::spin();      // 进入ROS事件循环
    return 0;
}