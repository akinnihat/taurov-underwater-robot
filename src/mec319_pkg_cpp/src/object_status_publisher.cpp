#include "rclcpp/rclcpp.hpp"
#include "mec319_interfaces/msg/object_coordinates.hpp"
#include "ObjectDetector.h"

class ObjectStatusPublisher : public rclcpp::Node 
{
public:
    ObjectStatusPublisher() : Node("object_status_publisher") 
    {
        this->object_detector = new ObjectDetector();
        publisher_ = this->create_publisher<mec319_interfaces::msg::ObjectCoordinates>(
            "object_status", 10);
        timer_ = this->create_wall_timer(std::chrono::microseconds(1), 
            std::bind(&ObjectStatusPublisher::publish_object_status, this));
        RCLCPP_INFO(this->get_logger(), "Object Status Publisher has been started.");
    }

private:
    ObjectDetector *object_detector;
  
    int m_xCoordinate;
    int m_yCoordinate;
    int m_radius;

private:
    void publish_object_status()
    {
        this->object_detector->RunFasterRCNN();
        auto msg = mec319_interfaces::msg::ObjectCoordinates();

        if (this->m_radius > 0) {msg.is_found = true;}
        else{msg.is_found = false;}

        msg.x_coordinate = this->object_detector->m_boundingBox.x;
        msg.y_coordinate = this->object_detector->m_boundingBox.y;
        msg.radius = this->object_detector->m_boundingBox.width;
        if (msg.is_found)
        {      
            RCLCPP_INFO(this->get_logger(), ("\nX Coordinate: " + std::to_string(this->m_xCoordinate) +
                                             "\nY Coordinate: " + std::to_string(this->m_yCoordinate) +
                                             "\nRadius: " + std::to_string(this->m_radius)));
        } else {RCLCPP_INFO(this->get_logger(), "Object not found.");}
        publisher_->publish(msg);
       
    }

    rclcpp::Publisher<mec319_interfaces::msg::ObjectCoordinates>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObjectStatusPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}