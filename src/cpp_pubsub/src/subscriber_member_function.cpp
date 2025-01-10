#include <memory>

#include <opencv2/opencv.hpp> // OpenCV库
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:
  /**
   * @brief 订阅者的代码整体流程和发布者类似，现在的节点名叫minimal_subscriber，构造函数中创建了订阅者，订阅String消息，订阅的话题名叫做“topic”，保存消息的队列长度是10，当订阅到数据时，会进入回调函数topic_callback。根据前面的教程，发布者和订阅者使用的话题名称和消息类型必须匹配才能进行通信。
    ————————————————
    版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。                        
    原文链接：https://blog.csdn.net/qq_29923461/article/details/120413799
   */
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    /**
     * @brief 回调函数中会收到String消息，然后并没有做太多处理，只是通过RCLCPP_INFO打印出来。
     * 
     */
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

class VideoSubscriber : public rclcpp::Node
{
  public:
    /**
     * @brief 订阅者的代码整体流程和发布者类似，现在的节点名叫Video_subscriber，构造函数中创建了订阅者，订阅String消息，订阅的话题名叫做“topic”，保存消息的队列长度是10，当订阅到数据时，会进入回调函数topic_callback。根据前面的教程，发布者和订阅者使用的话题名称和消息类型必须匹配才能进行通信。
        ————————————————
        版权声明：本文为博主原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接和本声明。                        
        原文链接：https://blog.csdn.net/qq_29923461/article/details/120413799
    */
    VideoSubscriber()
    : Node("Video_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Image>("video_frames", 10, std::bind(&VideoSubscriber::topic_callback, this, std::placeholders::_1));
    }

  private:
    /**
     * @brief 回调函数中会收到String消息，然后并没有做太多处理，只是通过RCLCPP_INFO打印出来。
     * 
     */
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg) const
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // 显示图像
        cv::imshow("Video Frame", cv_ptr->image);
        cv::waitKey(1);
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // 创建两个共享指针节点实例
    auto minimal_subscriber = std::make_shared<MinimalSubscriber>();
    auto video_subscriber = std::make_shared<VideoSubscriber>();

    // 为每个节点创建一个线程
    std::thread minimal_subscriber_thread([&]() {
        rclcpp::spin(minimal_subscriber);
    });

    std::thread video_subscriber_thread([&]() {
        rclcpp::spin(video_subscriber);
    });

    // 等待线程完成
    minimal_subscriber_thread.join();
    video_subscriber_thread.join();

    rclcpp::shutdown();
    return 0;
}

