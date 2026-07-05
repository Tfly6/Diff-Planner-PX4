#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/empty.hpp>
#include <traj_utils/msg/minco_traj.hpp>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <unistd.h>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#define UDP_PORT 8081
#define BUF_LEN 1048576

using namespace std;

enum MESSAGE_TYPE
{
  ODOM = 100,
  ONE_TRAJ,
  STOP,
  GOAL,
  JOY
};

class SwarmBridgeUDP : public rclcpp::Node
{
public:
  SwarmBridgeUDP() : Node("swarm_bridge")
  {
    this->declare_parameter<string>("broadcast_ip", "127.0.0.255");
    this->declare_parameter<int>("drone_id", -1);
    this->declare_parameter<double>("odom_max_freq", 1000.0);

    udp_ip_ = this->get_parameter("broadcast_ip").as_string();
    drone_id_ = this->get_parameter("drone_id").as_int();
    odom_broadcast_freq_ = this->get_parameter("odom_max_freq").as_double();

    if (drone_id_ == -1)
    {
      RCLCPP_WARN(this->get_logger(), "[swarm bridge] Wrong drone_id!");
      rclcpp::shutdown();
      return;
    }

    other_odoms_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "my_odom", 10, std::bind(&SwarmBridgeUDP::odom_sub_udp_cb, this, std::placeholders::_1));
    other_odoms_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/others_odom", 10);

    one_traj_sub_ = this->create_subscription<traj_utils::msg::MINCOTraj>(
      "/broadcast_traj_from_planner", 100, std::bind(&SwarmBridgeUDP::one_traj_sub_udp_cb, this, std::placeholders::_1));
    one_traj_pub_ = this->create_publisher<traj_utils::msg::MINCOTraj>("/broadcast_traj_to_planner", 100);

    goal_sub_ = this->create_subscription<quadrotor_msgs::msg::GoalSet>(
      "/goal_user2brig", 100, std::bind(&SwarmBridgeUDP::goal_sub_udp_cb, this, std::placeholders::_1));
    goal_pub_ = this->create_publisher<quadrotor_msgs::msg::GoalSet>("/goal_brig2plner", 100);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joystick_from_users", 100, std::bind(&SwarmBridgeUDP::joy_sub_udp_cb, this, std::placeholders::_1));
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joystick_from_bridge", 100);

    udp_send_fd_ = init_broadcast(udp_ip_.c_str(), UDP_PORT);
    cout << "[rosmsg_bridge_udp] start running" << endl;
    recv_thread_ = std::thread(&SwarmBridgeUDP::udp_recv_fun, this);
  }

  ~SwarmBridgeUDP()
  {
    if (recv_thread_.joinable())
    {
      pthread_cancel(recv_thread_.native_handle());
      recv_thread_.join();
    }
    close(udp_server_fd_);
    close(udp_send_fd_);
  }

private:
  int udp_server_fd_, udp_send_fd_;
  string udp_ip_;
  int drone_id_;
  double odom_broadcast_freq_;
  char udp_recv_buf_[BUF_LEN], udp_send_buf_[BUF_LEN];
  struct sockaddr_in addr_udp_send_;
  std::thread recv_thread_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr other_odoms_sub_;
  rclcpp::Subscription<traj_utils::msg::MINCOTraj>::SharedPtr one_traj_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::GoalSet>::SharedPtr goal_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr other_odoms_pub_;
  rclcpp::Publisher<traj_utils::msg::MINCOTraj>::SharedPtr one_traj_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::GoalSet>::SharedPtr goal_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;

  int init_broadcast(const char *ip, const int port)
  {
    int fd;
    if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "[bridge_node]Socket sender creation error!");
      exit(EXIT_FAILURE);
    }
    int so_broadcast = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &so_broadcast, sizeof(so_broadcast)) < 0)
    {
      cout << "Error in setting Broadcast option";
      exit(EXIT_FAILURE);
    }
    addr_udp_send_.sin_family = AF_INET;
    addr_udp_send_.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &addr_udp_send_.sin_addr) <= 0)
    {
      printf("\\nInvalid address/ Address not supported \\n");
      return -1;
    }
    return fd;
  }

  int udp_bind_to_port(const int port, int &server_fd)
  {
    struct sockaddr_in address;
    int opt = 1;
    if ((server_fd = socket(AF_INET, SOCK_DGRAM, 0)) == 0)
    {
      perror("socket failed");
      exit(EXIT_FAILURE);
    }
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
    {
      perror("setsockopt");
      exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
      perror("bind failed");
      exit(EXIT_FAILURE);
    }
    return server_fd;
  }

  template <typename T>
  int serializeTopic(const MESSAGE_TYPE msg_type, const T &msg)
  {
    auto ptr = (uint8_t *)(udp_send_buf_);
    *((MESSAGE_TYPE*)ptr) = msg_type;
    ptr += sizeof(MESSAGE_TYPE);

    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&msg, &serialized_msg);
    auto& rmw_msg = serialized_msg.get_rcl_serialized_message();
    uint32_t msg_size = rmw_msg.buffer_length;

    *((uint32_t *)ptr) = msg_size;
    ptr += sizeof(uint32_t);

    memcpy(ptr, rmw_msg.buffer, msg_size);
    return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
  }

  template <typename T>
  int deserializeTopic(T &msg)
  {
    auto ptr = (uint8_t *)(udp_recv_buf_ + sizeof(MESSAGE_TYPE));
    uint32_t msg_size = *((uint32_t *)ptr);
    ptr += sizeof(uint32_t);

    rclcpp::Serialization<T> serializer;
    rclcpp::SerializedMessage serialized_msg;
    auto& rmw_msg = serialized_msg.get_rcl_serialized_message();
    rmw_msg.buffer = ptr;
    rmw_msg.buffer_length = msg_size;

    serializer.deserialize_message(&serialized_msg, &msg);
    return msg_size + sizeof(MESSAGE_TYPE) + sizeof(uint32_t);
  }

  void odom_sub_udp_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    static rclcpp::Time t_last;
    rclcpp::Time t_now = this->now();
    if (t_last.nanoseconds() != 0 &&
        (t_now - t_last).seconds() * odom_broadcast_freq_ < 1.0)
    {
      return;
    }
    t_last = t_now;

    nav_msgs::msg::Odometry modified_msg = *msg;
    modified_msg.child_frame_id = string("drone_") + std::to_string(drone_id_);

    int len = serializeTopic(MESSAGE_TYPE::ODOM, modified_msg);
    if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
               (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "UDP SEND ERROR (1)!!!");
    }
  }

  void one_traj_sub_udp_cb(const traj_utils::msg::MINCOTraj::ConstSharedPtr &msg)
  {
    int len = serializeTopic(MESSAGE_TYPE::ONE_TRAJ, *msg);
    if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
               (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "UDP SEND ERROR (2)!!!");
    }
  }

  void goal_sub_udp_cb(const quadrotor_msgs::msg::GoalSet::ConstSharedPtr &msg)
  {
    int len = serializeTopic(MESSAGE_TYPE::GOAL, *msg);
    if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
               (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "UDP SEND ERROR (4)!!!");
    }
  }

  void joy_sub_udp_cb(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
  {
    int len = serializeTopic(MESSAGE_TYPE::JOY, *msg);
    if (sendto(udp_send_fd_, udp_send_buf_, len, 0,
               (struct sockaddr *)&addr_udp_send_, sizeof(addr_udp_send_)) <= 0)
    {
      RCLCPP_ERROR(this->get_logger(), "UDP SEND ERROR (5)!!!");
    }
  }

  void udp_recv_fun()
  {
    int valread;
    struct sockaddr_in addr_client;
    socklen_t addr_len;

    if (udp_bind_to_port(UDP_PORT, udp_server_fd_) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "[bridge_node]Socket recever creation error!");
      exit(EXIT_FAILURE);
    }

    nav_msgs::msg::Odometry odom_msg;
    traj_utils::msg::MINCOTraj MINCOTraj_msg;
    std_msgs::msg::Empty stop_msg;
    quadrotor_msgs::msg::GoalSet goal_msg;
    sensor_msgs::msg::Joy joy_msg;

    while (rclcpp::ok())
    {
      if ((valread = recvfrom(udp_server_fd_, udp_recv_buf_, BUF_LEN, 0,
                              (struct sockaddr *)&addr_client,
                              (socklen_t *)&addr_len)) < 0)
      {
        perror("recvfrom() < 0, error:");
        exit(EXIT_FAILURE);
      }

      switch (*((MESSAGE_TYPE *)udp_recv_buf_))
      {
        case MESSAGE_TYPE::ODOM:
        {
          if (valread == deserializeTopic(odom_msg))
            other_odoms_pub_->publish(odom_msg);
          else
            RCLCPP_ERROR(this->get_logger(), "Received message length not matches (ODOM)!!!");
          break;
        }
        case MESSAGE_TYPE::ONE_TRAJ:
        {
          if (valread == deserializeTopic(MINCOTraj_msg))
            one_traj_pub_->publish(MINCOTraj_msg);
          else
            RCLCPP_ERROR(this->get_logger(), "Received message length not matches (TRAJ)!!!");
          break;
        }
        case MESSAGE_TYPE::STOP:
        {
          if (valread == deserializeTopic(stop_msg))
            {}
          else
            RCLCPP_ERROR(this->get_logger(), "Received message length not matches (STOP)!!!");
          break;
        }
        case MESSAGE_TYPE::GOAL:
        {
          if (valread == deserializeTopic(goal_msg))
            goal_pub_->publish(goal_msg);
          else
            RCLCPP_ERROR(this->get_logger(), "Received message length not matches (GOAL)!!!");
          break;
        }
        case MESSAGE_TYPE::JOY:
        {
          if (valread == deserializeTopic(joy_msg))
            joy_pub_->publish(joy_msg);
          else
            RCLCPP_ERROR(this->get_logger(), "Received message length not matches (JOY)!!!");
          break;
        }
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown received message type???");
          break;
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SwarmBridgeUDP>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
