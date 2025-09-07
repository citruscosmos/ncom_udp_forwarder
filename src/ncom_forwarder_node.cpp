/**
 * @file ncom_forwarder_node.cpp
 * @brief A ROS2 node to forward NCOM data from a topic to a UDP port for NAVdisplay.
 * @version 0.1
 * @date 2025-09-06
 *
 * This node subscribes to an oxts_msgs/msg/Ncom topic and sends the raw NCOM
 * data payload as a UDP packet to a specified IP address and port. This is useful
 * for replaying rosbag data and visualizing it in real-time with NAVdisplay.
 *
 */
#include "rclcpp/rclcpp.hpp"
#include "oxts_msgs/msg/ncom.hpp"

#include <string>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

/**
 * @class NcomUdpForwarder
 * @brief Subscribes to NCOM ROS2 topic and publishes data over UDP.
 */
class NcomUdpForwarder : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Ncom Udp Forwarder object
     */
    NcomUdpForwarder()
        : Node("ncom_udp_forwarder")
    {
        // Declare and get parameters
        this->declare_parameter<std::string>("ncom_topic", "/oxts/ncom");
        this->declare_parameter<int>("udp_port", 3000);
        this->declare_parameter<std::string>("udp_address", "127.0.0.1"); // Add parameter for UDP address
        this->get_parameter("ncom_topic", ncom_topic_);
        this->get_parameter("udp_port", udp_port_);
        this->get_parameter("udp_address", udp_address_); // Get the address

        RCLCPP_INFO(this->get_logger(), "Forwarding NCOM data from topic '%s' to %s:%d", ncom_topic_.c_str(), udp_address_.c_str(), udp_port_);

        // Set up UDP socket
        setup_udp_socket();

        // Create subscriber
        subscription_ = this->create_subscription<oxts_msgs::msg::Ncom>(
            ncom_topic_, 10, std::bind(&NcomUdpForwarder::topic_callback, this, std::placeholders::_1));
    }

    /**
     * @brief Destroy the Ncom Udp Forwarder object
     */
    ~NcomUdpForwarder()
    {
        if (sockfd_ != -1)
        {
            close(sockfd_);
        }
    }

private:
    /**
     * @brief Set up the UDP socket for sending data.
     */
    void setup_udp_socket()
    {
        // Create UDP socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket.");
            rclcpp::shutdown();
            return;
        }

        // Configure server address
        memset(&server_addr_, 0, sizeof(server_addr_));
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(udp_port_);
        // Use the configured address
        server_addr_.sin_addr.s_addr = inet_addr(udp_address_.c_str()); 
    }

    /**
     * @brief Callback function for the NCOM topic subscriber.
     * @param msg The received NCOM message.
     */
    void topic_callback(const oxts_msgs::msg::Ncom::SharedPtr msg)
    {
        if (msg->raw_packet.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty NCOM data. Skipping.");
            return;
        }

        // Send the raw NCOM data over UDP
        ssize_t bytes_sent = sendto(sockfd_, msg->raw_packet.data(), msg->raw_packet.size(), 0,
                                    (const struct sockaddr *)&server_addr_, sizeof(server_addr_));

        if (bytes_sent < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send UDP packet.");
        }
        else if (static_cast<size_t>(bytes_sent) != msg->raw_packet.size())
        {
             RCLCPP_WARN(this->get_logger(), "Incomplete UDP packet sent. Sent %ld bytes, expected %zu bytes.", bytes_sent, msg->raw_packet.size());
        }
        else
        {
            RCLCPP_DEBUG(this->get_logger(), "Successfully forwarded NCOM packet of size %zu bytes.", msg->raw_packet.size());
        }
    }

    // ROS related
    rclcpp::Subscription<oxts_msgs::msg::Ncom>::SharedPtr subscription_;
    std::string ncom_topic_;

    // UDP related
    int sockfd_;
    int udp_port_;
    std::string udp_address_; // Add member variable for address
    struct sockaddr_in server_addr_;
};

/**
 * @brief Main function to initialize and run the node.
 * @param argc Argument count.
 * @param argv Argument vector.
 * @return int Exit code.
 */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NcomUdpForwarder>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}



