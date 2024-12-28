#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "sensor_msgs/msg/image.hpp" // 用于图像消息
#include <opencv2/opencv.hpp> // OpenCV库
#include "cv_bridge/cv_bridge.h" // 用于OpenCV和ROS图像之间的转换

#include "rclcpp/rclcpp.hpp" //ROS2中常用C++接口的头文件，使用C++编写的ROS2节点程序一定需要包含该头文件
#include "std_msgs/msg/string.hpp" //ROS2中字符串消息的头文件，后边我们会周期发布一个HelloWorld的字符串消息，所以需要包含该头文件。

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/**
 * @brief 通过继承rclcpp::Node节点基类创建节点类MinimalPublisher。代码中的每个this都引用节点。
 */
class MinimalPublisher : public rclcpp::Node
{
  public:

    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10); //话题名是topic,话题消息是string,保存消息队列长度是10
      timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this)); //500ms定时


    }

  private:
    /**
     * @brief 每次触发都会发布一次话题消息。message中保存的字符串是Hello world加一个计数值
     * ,然后通过RCLCPP_INFO宏函数打印一次日志信息，再通过发布者的publish方法将消息发布出去。
     * @details  //timer_callback是这里的关键，每次触发都会发布一次话题消息。
     * message中保存的字符串是Hello world加一个计数值，然后通过RCLCPP_INFO宏函数打印一次日志信息，再通过发布者的publish方法将消息发布出去。
     */
    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
    
    //定时器,发布者,计数器字段的声明
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    //cv::VideoCapture cap_;//opencv 视频捕获对象
  };

/**
 * @brief 通过继承rclcpp::Node节点基类创建节点类VideoPublisher。按照 @MinimalPublisher 编写
 */
class VideoPublisher : public rclcpp::Node
{
  public:

    VideoPublisher()
    : Node("video_publisher")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::Image>("video_frames", 10); //话题名是topic,话题消息是string,保存消息队列长度是10
      timer_ = this->create_wall_timer(40ms, std::bind(&VideoPublisher::timer_callback, this)); //500ms定时
      cap_.open("/home/zychen/Videos/video_example.mp4");
      if(!cap_.isOpened()){
        RCLCPP_ERROR(this->get_logger(),"failed to open video");
      }
    }

  private:
    /**
     * @brief 每次触发都会发布一次话题消息。message中保存的字符串是Hello world加一个计数值
     * ,然后通过RCLCPP_INFO宏函数打印一次日志信息，再通过发布者的publish方法将消息发布出去。
     * @details  //timer_callback是这里的关键，每次触发都会发布一次话题消息。
     * message中保存的字符串是Hello world加一个计数值，然后通过RCLCPP_INFO宏函数打印一次日志信息，再通过发布者的publish方法将消息发布出去。
     */
    void timer_callback()
    {
      cv::Mat frame;
      if (cap_.read(frame)) {
          auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
          publisher_->publish(*msg);
          RCLCPP_INFO(this->get_logger(), "Publishing video frame.");
      } else {
          RCLCPP_INFO(this->get_logger(), "End of video stream.");
          cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 重置视频流
      }
    }
    
    //定时器,发布者,计数器字段的声明
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_; //changed
    //opencv 视频捕获对象
    cv::VideoCapture cap_;
  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisher>());
    rclcpp::shutdown();
    return 0;
  }
