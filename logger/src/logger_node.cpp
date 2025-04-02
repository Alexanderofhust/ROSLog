#include "logger_node.h"
//#include <ros/ros.h>

// 构造函数
LoggerNode::LoggerNode() {
    // 初始化节点
    ros::NodeHandle nh("~");

    // 从参数服务器加载配置
    nh.getParam("topics", topics_);
    nh.param<std::string>("log_comment", log_comment_, "Log: ");
    nh.param<std::string>("log_path", log_path_, "/home/hustlyrm/roslog/mylogs/");  // 加载日志路径

    // 初始化订阅器
    for (const auto& topic : topics_) {
        subscribers_[topic] = nh.subscribe<std_msgs::String>(
            topic, 10, boost::bind(&LoggerNode::callback, this, _1, topic));
        topic_data_[topic] = "N/A";  // 初始化数据
    }

    // 创建定时器（1秒周期）
    timer_ = nh.createTimer(ros::Duration(1.0), &LoggerNode::timerCallback, this);

    std::string log_filename = getCurrentTimeString() + ".log";
    std::string log_path = log_path_  + log_filename;

    log_file_.open(log_path, std::ios::app);
    ROS_INFO("Log path: %s", log_path.c_str());
    if (!log_file_.is_open()) {
        ROS_ERROR("Failed to open log file!");
    }
}

LoggerNode::~LoggerNode(){
    
    // 关闭日志文件
    if (log_file_.is_open()) {
        log_file_.close();
    }
    
}

// 回调函数，处理订阅的消息
void LoggerNode::callback(const std_msgs::String::ConstPtr& msg, const std::string& topic) {
    topic_data_[topic] = msg->data;
}

// 定时器回调函数，记录日志
void LoggerNode::timerCallback(const ros::TimerEvent& event) {
    // 动态获取最新评论（允许实时调整）
    ros::param::get("~log_comment", log_comment_);
    
    // 获取系统资源使用情况
    double cpuUsage = getCpuUsage();
    unsigned long totalMem = 0, freeMem = 0, availableMem = 0;
    getMemoryUsage(totalMem, freeMem, availableMem);
    
    // 计算内存使用百分比
    double memUsagePercent = (totalMem - availableMem) * 100.0 / totalMem;
    double totalMemMB = totalMem / 1024.0;
    double usedMemMB = (totalMem - availableMem) / 1024.0;

    // 拼接所有话题数据和系统资源信息
    std::stringstream log_ss;
    log_ss << log_comment_;
    
    // 添加系统资源信息
    log_ss << " [CPU: " << std::fixed << std::setprecision(1) << cpuUsage << "%"
           << " | Mem: " << std::fixed << std::setprecision(1) << memUsagePercent << "%"
           << " (" << std::fixed << std::setprecision(1) << usedMemMB << "MB/" 
           << std::fixed << std::setprecision(1) << totalMemMB << "MB)]";
    
    // 添加各话题数据
    for (const auto& topic : topics_) {
        log_ss << " " << topic << ": " << topic_data_[topic];
    }

    std::string log_str = log_ss.str();

    // 输出日志到文件
    if (log_file_.is_open()) {
        log_file_ << log_str << std::endl;
    }

    // 输出到ROS日志系统
    ROS_INFO("%s", log_str.c_str());
}

void LoggerNode::recordSystemStatus()
{

}

// 获取系统时间戳，当前不需要
// std::string getCurrentTimestamp() {
//     auto now = std::chrono::system_clock::now();
//     auto in_time_t = std::chrono::system_clock::to_time_t(now);
//     std::stringstream ss;
//     ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
//     return ss.str();
// }

// 获取CPU使用率
double LoggerNode::getCpuUsage() {
    static unsigned long long lastTotalUser = 0, lastTotalUserLow = 0, lastTotalSys = 0, lastTotalIdle = 0;
    
    std::ifstream procStat("/proc/stat");
    std::string line;
    std::getline(procStat, line);
    std::istringstream iss(line);
    
    std::string cpu;
    unsigned long long user, nice, system, idle, iowait, irq, softirq, steal, guest, guest_nice;
    iss >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal >> guest >> guest_nice;
    
    unsigned long long totalIdle = idle + iowait;
    unsigned long long totalNonIdle = user + nice + system + irq + softirq + steal;
    unsigned long long total = totalIdle + totalNonIdle;
    
    static double prevCpuUsage = 0.0;
    
    if (lastTotalUser != 0) {
        unsigned long long totald = total - (lastTotalUser + lastTotalSys + lastTotalIdle);
        unsigned long long idled = totalIdle - lastTotalIdle;
        double cpuUsage = (totald - idled) * 100.0 / totald;
        prevCpuUsage = cpuUsage;
    }
    
    lastTotalUser = user;
    lastTotalSys = system;
    lastTotalIdle = totalIdle;
    
    return prevCpuUsage;
}

// 获取内存使用情况
void LoggerNode::getMemoryUsage(unsigned long& total, unsigned long& free, unsigned long& available) {
    std::ifstream memInfo("/proc/meminfo");
    std::string line;
    
    while (std::getline(memInfo, line)) {
        std::istringstream iss(line);
        std::string key;
        unsigned long value;
        std::string unit;
        
        iss >> key >> value >> unit;
        
        if (key == "MemTotal:") {
            total = value;
        } else if (key == "MemFree:") {
            free = value;
        } else if (key == "MemAvailable:") {
            available = value;
        }
    }
}

// 主函数
int main(int argc, char** argv) {
    ros::init(argc, argv, "logger_node");
    LoggerNode node;  // 创建LoggerNode实例
    ros::spin();      // 进入ROS事件循环
    return 0;
}