#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <vector>
#include <queue>
#include <fstream>
#include <yaml-cpp/yaml.h>

class CloudToGridMapNode : public rclcpp::Node {
public:
    CloudToGridMapNode() : Node("local_map_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        // Load map parameters from map.yaml
        std::string map_dir = "/home/takrop/ros2_ws/src/jimbot_navigation/map";
        std::string yaml_file = map_dir + "/won1_2.yaml";

        
        YAML::Node map_metadata = YAML::LoadFile(yaml_file);
        resolution_ = map_metadata["resolution"].as<float>();
        origin_x_ = map_metadata["origin"][0].as<float>();
        origin_y_ = map_metadata["origin"][1].as<float>();

        // Load map image to determine width and height
        std::string map_image_path = map_dir + "/" + map_metadata["image"].as<std::string>();
        std::ifstream map_image(map_image_path, std::ios::binary);
        if (!map_image.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open map image: %s", map_image_path.c_str());
            throw std::runtime_error("Failed to open map image");
        }

        // Assuming the map is in PGM format, extract width and height from the header
        std::string line;
        std::getline(map_image, line); // PGM header (e.g., "P5")
        std::getline(map_image, line); // Comment line
        // map_image >> width_ >> height_; // Width and height

        // make map_image to grid
        map_image.seekg(0, std::ios::end);
        size_t file_size = map_image.tellg();
        std::vector<uint8_t> image_data(file_size);
        map_image.read(reinterpret_cast<char*>(image_data.data()), file_size);
        map_image.close();

        initialize_grid_from_map(map_grid_, map_dir);

        RCLCPP_INFO(this->get_logger(), "Loaded map: resolution=%.2f, width=%d, height=%d, origin=(%.2f, %.2f)",
                    resolution_, width_, height_, origin_x_, origin_y_);

        // Declare additional parameters
        this->declare_parameter<int>("inflation_radius", 3);
        inflation_radius_ = this->get_parameter("inflation_radius").as_int();

        // PointCloud2 topic subscription
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/merged_cloud", 2,
            std::bind(&CloudToGridMapNode::pointcloud_callback, this, std::placeholders::_1)
        );

        // OccupancyGrid publisher
        grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/local_costmap", 1);
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Initialize grid from map
        // std::vector<int8_t> map_grid(width_ * height_, -1);  // Grid initialized from map
        // initialize_grid_from_map(map_grid);

        // Create a grid for the current point cloud
        std::vector<int8_t> cloud_grid(width_ * height_, -1);  // -1: Unknown

        // RCLCPP_INFO(this->get_logger(), "Received PointCloud2 with %d points", msg->width * msg->height);

        // Get the transform from base_footprint to map
        geometry_msgs::msg::TransformStamped transform_stamped;
        bool transform_flag = false;
        try {
            transform_stamped = tf_buffer_.lookupTransform(map_frame, "base_footprint", tf2::TimePointZero);
            transform_flag = true;
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform base_footprint to map: %s", ex.what());
        }

        // Iterate through PointCloud2 data and map obstacles
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        int valid_points = 0;
        while (iter_x != iter_x.end()) {
            float x, y, z;

            x = *iter_x;
            y = *iter_y;
            z = *iter_z;

            if (transform_flag) {
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.point.x = x;
                point_in.point.y = y;
                point_in.point.z = z;
                point_in.header.frame_id = msg->header.frame_id;

                try {
                    point_out = tf_buffer_.transform<geometry_msgs::msg::PointStamped>(
                        point_in, map_frame, tf2::durationFromSec(0.1));
                    x = point_out.point.x;
                    y = point_out.point.y;
                } catch (tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "Failed to transform point: %s", ex.what());
                    ++iter_x;
                    ++iter_y;
                    ++iter_z;
                    continue;
                }
            }

            int mx = static_cast<int>((x - origin_x_) / resolution_);
            int my = static_cast<int>((y - origin_y_) / resolution_);

            if (0 <= mx && mx < width_ && 0 <= my && my < height_) {
                cloud_grid[my * width_ + mx] = 100;  // Obstacle
                valid_points++;
            }

            ++iter_x;
            ++iter_y;
            ++iter_z;
        }

        RCLCPP_INFO(this->get_logger(), "Processed %d valid points into the grid map", valid_points);

        // Apply inflation to the cloud grid
        apply_inflation(cloud_grid);

        // Merge the cloud grid with the map grid
        std::vector<int8_t> merged_grid = merge_grids(map_grid_, cloud_grid);

        // Publish the merged grid map
        publish_grid(merged_grid);
    }

    void initialize_grid_from_map(std::vector<int8_t>& grid, std::string map_dir) {
        // Load map image to determine initial grid values
        std::string yaml_file = map_dir + "/map.yaml";

        YAML::Node map_metadata = YAML::LoadFile(yaml_file);
        std::string map_image_path = map_dir + "/" + map_metadata["image"].as<std::string>();

        std::ifstream map_image(map_image_path, std::ios::binary);
        if (!map_image.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open map image: %s", map_image_path.c_str());
            throw std::runtime_error("Failed to open map image");
        }

        // Skip PGM header
        std::string line;
        std::getline(map_image, line); // PGM header (e.g., "P5")
        std::getline(map_image, line); // Comment line
        // map_image >> width_ >> height_; // Width and height
        map_image.ignore(); // Skip the newline after width and height
        map_image.ignore(); // Skip the newline after max value

        // Read pixel data and initialize the grid
        grid.resize(width_ * height_);
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                unsigned char pixel;
                map_image.read(reinterpret_cast<char*>(&pixel), 1);

                // Convert pixel value to occupancy grid value
                if (pixel <= 10) {
                    grid[y * width_ + x] = 100; // Occupied
                } else if (pixel >= 250) {
                    grid[y * width_ + x] = 0; // Free
                } else {
                    grid[y * width_ + x] = -1; // Unknown
                }
            }
        }

        map_image.close();
        RCLCPP_INFO(this->get_logger(), "Grid initialized from map image: width=%d, height=%d", width_, height_);
    }

    std::vector<int8_t> merge_grids(const std::vector<int8_t>& map_grid, const std::vector<int8_t>& cloud_grid) {
        std::vector<int8_t> merged_grid(width_ * height_, -1);

        for (int i = 0; i < width_ * height_; ++i) {
            if (cloud_grid[i] == 100) {
                merged_grid[i] = 100; // Obstacle from cloud
            } else if (map_grid[i] == 100) {
                merged_grid[i] = 100; // Obstacle from map
            } else if (map_grid[i] == 0 || cloud_grid[i] == 0) {
                merged_grid[i] = 0; // Free space
            } else {
                merged_grid[i] = -1; // Unknown
            }
        }

        return merged_grid;
    }

    void apply_inflation(std::vector<int8_t>& grid) {
        std::queue<std::pair<int, int>> inflation_queue;

        // Add obstacles to the queue
        for (int i = 0; i < width_; ++i) {
            for (int j = 0; j < height_; ++j) {
                if (grid[j * width_ + i] == 100) {
                    inflation_queue.push({i, j});
                }
            }
        }

        // Expand obstacles within the inflation radius
        while (!inflation_queue.empty()) {
            auto [x, y] = inflation_queue.front();
            inflation_queue.pop();

            for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx) {
                for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy) {
                    int nx = x + dx;
                    int ny = y + dy;

                    if (nx >= 0 && ny >= 0 && nx < width_ && ny < height_) {
                        if (std::sqrt(dx * dx + dy * dy) <= inflation_radius_) {
                            grid[ny * width_ + nx] = 100;  // Inflate obstacle
                        }
                    }
                }
            }
        }
    }

    void publish_grid(const std::vector<int8_t>& grid_data) {
        auto msg = nav_msgs::msg::OccupancyGrid();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = map_frame;
        msg.info.resolution = resolution_;
        msg.info.width = width_;
        msg.info.height = height_;
        msg.info.origin.position.x = origin_x_;
        msg.info.origin.position.y = origin_y_;
        msg.info.origin.orientation.w = 1.0;
        msg.data = grid_data;

        // Debugging: Check if the grid data is being populated
        // int obstacle_count = std::count(grid_data.begin(), grid_data.end(), 100);
        // int unknown_count = std::count(grid_data.begin(), grid_data.end(), -1);
        // RCLCPP_INFO(this->get_logger(), "Grid map published: Obstacles=%d, Unknown=%d, Total=%ld",
        //             obstacle_count, unknown_count, grid_data.size());

        grid_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::vector<int8_t> map_grid_;

    float resolution_;
    int width_;
    int height_;
    float origin_x_;
    float origin_y_;
    int inflation_radius_;
    std::string map_frame = "odom";
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudToGridMapNode>());
    rclcpp::shutdown();
    return 0;
}