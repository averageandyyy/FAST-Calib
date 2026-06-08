/* 
Developer: Chunran Zheng <zhengcr@connect.hku.hk>

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef DATA_PREPROCESS_HPP
#define DATA_PREPROCESS_HPP

#include <Eigen/Core>
#include <cstring>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <fstream>
#include "common_lib.h"

using namespace std;

enum class LiDARType : int {
    Unknown = 0,
    Solid   = 1,   // 固态（如 Livox）
    Mech    = 2    // 机械式多线
};

static const sensor_msgs::msg::PointField* findPointField(
    const sensor_msgs::msg::PointCloud2 &msg,
    const std::string &name)
{
    for (const auto &field : msg.fields)
    {
        if (field.name == name) return &field;
    }
    return nullptr;
}

static std::uint16_t readRingValue(
    const sensor_msgs::msg::PointCloud2 &msg,
    const sensor_msgs::msg::PointField &ring_field,
    size_t point_index)
{
    const size_t offset = point_index * msg.point_step + ring_field.offset;
    const auto *data = msg.data.data() + offset;

    switch (ring_field.datatype)
    {
        case sensor_msgs::msg::PointField::UINT8:
            return static_cast<std::uint16_t>(*data);
        case sensor_msgs::msg::PointField::INT8:
            return static_cast<std::uint16_t>(*reinterpret_cast<const std::int8_t*>(data));
        case sensor_msgs::msg::PointField::UINT16:
        {
            std::uint16_t value;
            std::memcpy(&value, data, sizeof(value));
            return value;
        }
        case sensor_msgs::msg::PointField::INT16:
        {
            std::int16_t value;
            std::memcpy(&value, data, sizeof(value));
            return static_cast<std::uint16_t>(value);
        }
        case sensor_msgs::msg::PointField::UINT32:
        {
            std::uint32_t value;
            std::memcpy(&value, data, sizeof(value));
            return static_cast<std::uint16_t>(value);
        }
        case sensor_msgs::msg::PointField::INT32:
        {
            std::int32_t value;
            std::memcpy(&value, data, sizeof(value));
            return static_cast<std::uint16_t>(value);
        }
        default:
            return 0xFFFF;
    }
}

class DataPreprocess
{
public:
    // 改成带线号的点云
    pcl::PointCloud<Common::Point>::Ptr cloud_input_;
    cv::Mat img_input_;
    LiDARType lidar_type_{LiDARType::Unknown};
    LiDARType lidarType() const { return lidar_type_; }

    DataPreprocess(const rclcpp::Node::SharedPtr &node, Params &params)
        : cloud_input_(new pcl::PointCloud<Common::Point>)
    {
        string bag_path   = params.bag_path;
        string image_path = params.image_path;
        string lidar_topic = params.lidar_topic;

        // 读图像
        img_input_ = cv::imread(image_path, cv::IMREAD_UNCHANGED);
        if (img_input_.empty())
        {
            std::string msg = "Loading the image " + image_path + " failed";
            RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
            return;
        }

        // ROS2 bags are usually directories that contain metadata.yaml and storage files.
        if (!std::filesystem::exists(bag_path))
        {
            std::string msg = "Loading the rosbag " + bag_path + " failed";
            RCLCPP_ERROR(node->get_logger(), "%s", msg.c_str());
            return;
        }
        RCLCPP_INFO(node->get_logger(), "Loading the rosbag %s", bag_path.c_str());

        rosbag2_cpp::Reader reader;
        try
        {
            reader.open(bag_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node->get_logger(), "LOADING BAG FAILED: %s", e.what());
            return;
        }

        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;

        // 累计读取
        while (reader.has_next())
        {
            auto bag_message = reader.read_next();
            if (bag_message->topic_name != lidar_topic)
            {
                continue;
            }

            sensor_msgs::msg::PointCloud2 pcl_msg;
            try
            {
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serializer.deserialize_message(&serialized_msg, &pcl_msg);
            }
            catch (const std::exception &e)
            {
                RCLCPP_WARN(node->get_logger(), "Skipping non-PointCloud2 message on %s: %s",
                    lidar_topic.c_str(), e.what());
                continue;
            }

            const auto *ring_field = findPointField(pcl_msg, "ring");
            const bool has_ring = ring_field != nullptr;
            lidar_type_ = has_ring ? LiDARType::Mech : LiDARType::Solid;

            sensor_msgs::PointCloud2ConstIterator<float> it_x(pcl_msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> it_y(pcl_msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> it_z(pcl_msg, "z");

            const size_t n = static_cast<size_t>(pcl_msg.width) * pcl_msg.height;
            cloud_input_->reserve(cloud_input_->size() + n);

            for (size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z)
            {
                Common::Point p;
                p.x = *it_x;
                p.y = *it_y;
                p.z = *it_z;
                p.ring = has_ring ? readRingValue(pcl_msg, *ring_field, i) : 0xFFFF;
                cloud_input_->push_back(p);
            }
        }

        RCLCPP_INFO(node->get_logger(), "Loaded %zu points from the rosbag.", cloud_input_->size());
    }
};

typedef std::shared_ptr<DataPreprocess> DataPreprocessPtr;

#endif // DATA_PREPROCESS_HPP