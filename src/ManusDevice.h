#pragma once

#include <mc_rbdyn/Device.h>
#include <mc_rtc/gui.h>
#include <SpaceVecAlg/SpaceVecAlg>

#include <chrono>
#include <mutex>
#include <optional>
#include <string>
//TODO: Check how to make the unordered_map work with ROS2 message
#include <unordered_map>
#include <vector>

#include <manus_ros2_msgs/msg/manus_glove.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mc_rbdyn
{
//home/champagnepapi/manus_ws/install/manus_ros2_msgs/include/manus_ros2_msgs/manus_ros2_msgs/msg/detail/manus_raw_node__struct.hpp
struct ManusDevice : public Device
{
  struct RawNode
  {
    int node_id{0};
    int parent_node_id{0};
    std::string joint_type;
    std::string chain_type;
    sva::PTransformd pose{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
  };

  struct Ergonomics
  {
    std::string type;
    double value{0.};
  };

  struct RawSensor
  {
    int sensorId{0};
    sva::PTransformd pose{Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()};
  };

  // I am taking the struct of ManusGlove.msg as a reference path: /home/champagnepapi/manus_ws/src/ROS2/manus_ros2_msgs/msg/ManusGlove.msg
  struct Data
  {
    //int glove_id{-1};
    int glove_id{0};
    std::string side;
    int raw_node_count{0};
    std::vector<RawNode> raw_nodes;
    int ergonomics_count{0};
    std::vector<Ergonomics> ergonomics;
    std::optional<Eigen::Quaterniond> wristOrientation;
    int raw_sensor_count{0};
    std::vector<RawSensor> raw_sensors;
    //TODO: change to map for easy access
    std::unordered_map<std::string, RawNode> finger;
    std::chrono::steady_clock::time_point stamp{};
  };

  ~ManusDevice() noexcept override = default;

  ManusDevice(const ManusDevice & other);
  ManusDevice & operator=(const ManusDevice & other);

  ManusDevice(ManusDevice &&) noexcept = default;
  ManusDevice & operator=(ManusDevice &&) noexcept = default;

  ManusDevice(const std::string & name);

  ManusDevice(const std::string & name, const std::string & topic, rclcpp::Node::SharedPtr node);

  DevicePtr clone() const override;

  const ManusDevice::Data & data() const;

  void addToGUI(mc_rtc::gui::StateBuilder & gui);

private:
  mutable std::mutex dataMutex_;
  Data data_;

  void gloveCallback(const manus_ros2_msgs::msg::ManusGlove::SharedPtr msg);

  std::string topic_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<manus_ros2_msgs::msg::ManusGlove>::SharedPtr sub_;
};

} // namespace mc_rbdyn
