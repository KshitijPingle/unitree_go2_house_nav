#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/**
 * NavigationNode implements a ROS2 navigation controller for Unitree GO2.
 *
 * Supported modes:
 *   - "goal": follow a straight-line path to a fixed target waypoint.
 *   - "explore": perform frontier-based exploration on an occupancy grid.
 *
 * The node subscribes to odometry, occupancy grid, and laser scan data, and
 * publishes velocity commands. Isaac Sim compatibility is supported via
 * configurable simulated topic namespaces.
 */
class NavigationNode : public rclcpp::Node
{
public:
  /**
   * Construct the navigation node, declare parameters, and initialize ROS2
   * publishers/subscribers and the main control timer.
   */
  NavigationNode() : Node("go2_navigation_node")
  {
    sim_mode_ = declare_parameter<bool>("sim_mode", false);
    isaac_namespace_ = declare_parameter<std::string>("isaac_namespace", "/isaac");

    mode_ = declare_parameter<std::string>("mode", "explore");  // "goal" or "explore"

    if (sim_mode_) {
      scan_topic_ = declare_parameter<std::string>("scan_topic", isaac_namespace_ + "/scan");
      odom_topic_ = declare_parameter<std::string>("odom_topic", isaac_namespace_ + "/odom");
      map_topic_ = declare_parameter<std::string>("map_topic", isaac_namespace_ + "/map");
      cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", isaac_namespace_ + "/cmd_vel");
    } else {
      scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
      odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
      map_topic_ = declare_parameter<std::string>("map_topic", "/map");
      cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    }
    target_x_ = declare_parameter<double>("target_x", 2.0);
    target_y_ = declare_parameter<double>("target_y", 0.0);

    goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.30);
    waypoint_spacing_ = declare_parameter<double>("waypoint_spacing", 0.60);
    coverage_spacing_cells_ = declare_parameter<int>("coverage_spacing_cells", 8);

    max_linear_speed_ = declare_parameter<double>("max_linear_speed", 0.30);
    max_angular_speed_ = declare_parameter<double>("max_angular_speed", 0.80);
    slow_down_distance_ = declare_parameter<double>("slow_down_distance", 0.75);

    obstacle_stop_distance_ = declare_parameter<double>("obstacle_stop_distance", 0.45);
    obstacle_slow_distance_ = declare_parameter<double>("obstacle_slow_distance", 0.80);

    occupied_threshold_ = declare_parameter<int>("occupied_threshold", 65);
    unknown_is_blocked_ = declare_parameter<bool>("unknown_is_blocked", true);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&NavigationNode::scanCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&NavigationNode::odomCallback, this, std::placeholders::_1));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, 10,
      std::bind(&NavigationNode::mapCallback, this, std::placeholders::_1));

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NavigationNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Navigation node started in mode='%s'.", mode_.c_str());
    RCLCPP_INFO(get_logger(), "sim_mode='%s'", sim_mode_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "odom_topic='%s'", odom_topic_.c_str());
    RCLCPP_INFO(get_logger(), "scan_topic='%s'", scan_topic_.c_str());
    RCLCPP_INFO(get_logger(), "map_topic='%s'", map_topic_.c_str());
    RCLCPP_INFO(get_logger(), "cmd_vel_topic='%s'", cmd_vel_topic_.c_str());
  }

private:
  struct Waypoint
  {
    double x;
    double y;
  };

  struct Cell
  {
    int x;
    int y;
  };

  static double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }

  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
  {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  int mapIndex(int x, int y) const
  {
    return y * static_cast<int>(latest_map_->info.width) + x;
  }

  bool inMap(int x, int y) const
  {
    return latest_map_ &&
      x >= 0 && y >= 0 &&
      x < static_cast<int>(latest_map_->info.width) &&
      y < static_cast<int>(latest_map_->info.height);
  }

  /**
   * Return true if a map cell is considered traversable.
   * Unknown cells are treated as free when unknown_is_blocked_ is false.
   */
  bool isFreeCell(int x, int y) const
  {
    if (!inMap(x, y)) return false;

    const int value = latest_map_->data[mapIndex(x, y)];

    if (value < 0) {
      return !unknown_is_blocked_;
    }

    return value < occupied_threshold_;
  }

  /**
   * Return true when a free cell borders unknown map space, making it a
   * candidate frontier cell for exploration.
   */
  bool isFrontierCell(int x, int y) const
  {
    if (!isFreeCell(x, y)) return false;

    static const int dx[4] = {1, -1, 0, 0};
    static const int dy[4] = {0, 0, 1, -1};

    for (int i = 0; i < 4; ++i) {
      const int nx = x + dx[i];
      const int ny = y + dy[i];
      if (!inMap(nx, ny)) continue;
      const int value = latest_map_->data[mapIndex(nx, ny)];
      if (value < 0) {
        return true;
      }
    }

    return false;
  }

  /**
   * Return the 4-connected neighbor cells for the given grid coordinate.
   */
  std::vector<Cell> getCellNeighbors(const Cell & cell) const
  {
    static const int dx[4] = {1, -1, 0, 0};
    static const int dy[4] = {0, 0, 1, -1};

    std::vector<Cell> neighbors;
    for (int i = 0; i < 4; ++i) {
      neighbors.push_back({cell.x + dx[i], cell.y + dy[i]});
    }
    return neighbors;
  }

  std::vector<Waypoint> findFrontierWaypoints() const
  {
    struct FrontierGoal
    {
      Waypoint waypoint;
      int travel_cost;
    };

    std::vector<Waypoint> frontiers;
    if (!latest_map_ || !have_odom_) {
      return frontiers;
    }

    const int width = static_cast<int>(latest_map_->info.width);
    const int height = static_cast<int>(latest_map_->info.height);
    if (width <= 0 || height <= 0) {
      return frontiers;
    }

    const Cell start = worldToCell(robot_x_, robot_y_);
    if (!inMap(start.x, start.y) || !isFreeCell(start.x, start.y)) {
      return frontiers;
    }

    std::vector<char> visited(width * height, 0);
    std::vector<char> frontier_mask(width * height, 0);
    std::vector<int> distance(width * height, -1);
    std::queue<Cell> queue;

    const int start_index = mapIndex(start.x, start.y);
    queue.push(start);
    visited[start_index] = 1;
    distance[start_index] = 0;

    std::vector<Cell> frontier_cells;

    while (!queue.empty()) {
      const Cell current = queue.front();
      queue.pop();
      const int current_index = mapIndex(current.x, current.y);

      if (isFrontierCell(current.x, current.y)) {
        frontier_cells.push_back(current);
        frontier_mask[current_index] = 1;
      }

      for (const Cell & neighbor : getCellNeighbors(current)) {
        if (!inMap(neighbor.x, neighbor.y)) {
          continue;
        }

        const int neighbor_index = mapIndex(neighbor.x, neighbor.y);
        if (visited[neighbor_index]) {
          continue;
        }

        if (!isFreeCell(neighbor.x, neighbor.y)) {
          continue;
        }

        visited[neighbor_index] = 1;
        distance[neighbor_index] = distance[current_index] + 1;
        queue.push(neighbor);
      }
    }

    if (frontier_cells.empty()) {
      return frontiers;
    }

    std::vector<char> clustered(width * height, 0);
    std::vector<FrontierGoal> goals;
    goals.reserve(32);

    for (const Cell & frontier_cell : frontier_cells) {
      const int index = mapIndex(frontier_cell.x, frontier_cell.y);
      if (clustered[index]) {
        continue;
      }

      int cluster_cost = std::numeric_limits<int>::max();
      Cell best_cell = frontier_cell;
      double sum_x = 0.0;
      double sum_y = 0.0;
      int count = 0;
      std::queue<Cell> cluster_queue;
      cluster_queue.push(frontier_cell);
      clustered[index] = 1;

      while (!cluster_queue.empty()) {
        const Cell current = cluster_queue.front();
        cluster_queue.pop();
        const int current_index = mapIndex(current.x, current.y);

        sum_x += static_cast<double>(current.x);
        sum_y += static_cast<double>(current.y);
        ++count;

        const int current_cost = distance[current_index];
        if (current_cost >= 0 && current_cost < cluster_cost) {
          cluster_cost = current_cost;
          best_cell = current;
        }

        for (const Cell & neighbor : getCellNeighbors(current)) {
          if (!inMap(neighbor.x, neighbor.y)) {
            continue;
          }

          const int neighbor_index = mapIndex(neighbor.x, neighbor.y);
          if (clustered[neighbor_index] || !frontier_mask[neighbor_index]) {
            continue;
          }

          clustered[neighbor_index] = 1;
          cluster_queue.push(neighbor);
        }
      }

      if (count > 0) {
        if (cluster_cost < 0) {
          cluster_cost = std::numeric_limits<int>::max();
        }
        goals.push_back({cellToWorld(best_cell.x, best_cell.y), cluster_cost});
      }
    }

    std::sort(goals.begin(), goals.end(), [this](const FrontierGoal & a, const FrontierGoal & b) {
      if (a.travel_cost != b.travel_cost) {
        return a.travel_cost < b.travel_cost;
      }
      return std::hypot(a.waypoint.x - robot_x_, a.waypoint.y - robot_y_) <
             std::hypot(b.waypoint.x - robot_x_, b.waypoint.y - robot_y_);
    });

    frontiers.reserve(goals.size());
    for (const FrontierGoal & goal : goals) {
      frontiers.push_back(goal.waypoint);

      RCLCPP_INFO(this->get_logger(),
        "New frontier goal selected: x=%.2f y=%.2f travel_cost=%d",
        goal.waypoint.x, goal.waypoint.y, goal.travel_cost);
    }

    return frontiers;
  }

  Cell worldToCell(double wx, double wy) const
  {
    const double origin_x = latest_map_->info.origin.position.x;
    const double origin_y = latest_map_->info.origin.position.y;
    const double res = latest_map_->info.resolution;

    return {
      static_cast<int>((wx - origin_x) / res),
      static_cast<int>((wy - origin_y) / res)
    };
  }

  Waypoint cellToWorld(int cx, int cy) const
  {
    const double origin_x = latest_map_->info.origin.position.x;
    const double origin_y = latest_map_->info.origin.position.y;
    const double res = latest_map_->info.resolution;

    return {
      origin_x + (static_cast<double>(cx) + 0.5) * res,
      origin_y + (static_cast<double>(cy) + 0.5) * res
    };
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_front_distance_ = std::numeric_limits<double>::infinity();
    latest_left_distance_ = std::numeric_limits<double>::infinity();
    latest_right_distance_ = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const double range = msg->ranges[i];
      if (!std::isfinite(range)) continue;

      const double angle = msg->angle_min + static_cast<double>(i) * msg->angle_increment;

      if (std::abs(angle) < M_PI / 6.0) {
        latest_front_distance_ = std::min(latest_front_distance_, range);
      } else if (angle >= M_PI / 6.0 && angle <= 5.0 * M_PI / 9.0) {
        latest_left_distance_ = std::min(latest_left_distance_, range);
      } else if (angle <= -M_PI / 6.0 && angle >= -5.0 * M_PI / 9.0) {
        latest_right_distance_ = std::min(latest_right_distance_, range);
      }
    }

    have_scan_ = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;
    robot_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    have_odom_ = true;
  }

  /**
   * Handle incoming map updates and trigger waypoint generation when
   * odometry is already available.
   */
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
    have_map_ = true;

    if (!have_odom_) return;

    if (!generated_waypoints_) {
      if (mode_ == "explore") {
        generateCoverageWaypoints();
      } else {
        generateStraightLineWaypoints();
      }
      generated_waypoints_ = true;
    }
  }

  /**
   * Create a simple straight-line sequence of intermediate waypoints between
   * the current robot pose and the configured target coordinate.
   */
  void generateStraightLineWaypoints()
  {
    waypoints_.clear();

    const double dx = target_x_ - robot_x_;
    const double dy = target_y_ - robot_y_;
    const double distance = std::hypot(dx, dy);
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / waypoint_spacing_)));

    for (int i = 1; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      waypoints_.push_back({robot_x_ + t * dx, robot_y_ + t * dy});
    }

    current_waypoint_index_ = 0;
    RCLCPP_INFO(get_logger(), "Generated %zu goal-navigation waypoints.", waypoints_.size());
  }

  /**
   * Generate exploration waypoints for the current map.
   * Tries frontier-based exploration first, and falls back to a simple
   * lawnmower coverage trajectory when no frontiers are available.
   */
  void generateCoverageWaypoints()
  {
    waypoints_ = findFrontierWaypoints();
    current_waypoint_index_ = 0;

    if (waypoints_.empty()) {
      RCLCPP_WARN(get_logger(), "No frontiers detected; falling back to lawnmower coverage.");
      waypoints_ = generateLawnmowerCoverageWaypoints();
      current_waypoint_index_ = 0;
      if (waypoints_.empty()) {
        RCLCPP_WARN(get_logger(), "Fallback coverage generation also produced no waypoints.");
      } else {
        RCLCPP_INFO(get_logger(), "Generated %zu fallback coverage waypoints.", waypoints_.size());
      }
    } else {
      RCLCPP_INFO(get_logger(), "Generated %zu frontier-based exploration waypoints.", waypoints_.size());
    }
  }

  /**
   * Generate a fallback lawnmower coverage path over free map cells.
   * This is used when frontier-based exploration does not find any goals.
   */
  std::vector<Waypoint> generateLawnmowerCoverageWaypoints() const
  {
    std::vector<Waypoint> coverage_waypoints;
    if (!latest_map_) {
      return coverage_waypoints;
    }

    const int width = static_cast<int>(latest_map_->info.width);
    const int height = static_cast<int>(latest_map_->info.height);
    const int spacing = std::max(1, coverage_spacing_cells_);

    bool left_to_right = true;

    for (int y = spacing; y < height - spacing; y += spacing) {
      std::vector<Cell> row_cells;

      if (left_to_right) {
        for (int x = spacing; x < width - spacing; x += spacing) {
          if (isFreeCell(x, y)) row_cells.push_back({x, y});
        }
      } else {
        for (int x = width - spacing - 1; x >= spacing; x -= spacing) {
          if (isFreeCell(x, y)) row_cells.push_back({x, y});
        }
      }

      for (const auto & cell : row_cells) {
        Waypoint wp = cellToWorld(cell.x, cell.y);
        if (coverage_waypoints.empty() ||
            std::hypot(wp.x - coverage_waypoints.back().x, wp.y - coverage_waypoints.back().y) > waypoint_spacing_) {
          coverage_waypoints.push_back(wp);
        }
      }

      left_to_right = !left_to_right;
    }

    if (!coverage_waypoints.empty()) {
      auto nearest_it = std::min_element(
        coverage_waypoints.begin(),
        coverage_waypoints.end(),
        [this](const Waypoint & a, const Waypoint & b) {
          return std::hypot(a.x - robot_x_, a.y - robot_y_) <
                 std::hypot(b.x - robot_x_, b.y - robot_y_);
        });
      std::rotate(coverage_waypoints.begin(), nearest_it, coverage_waypoints.end());
    }

    return coverage_waypoints;
  }

  bool waypointReached() const
  {
    if (waypoints_.empty() || current_waypoint_index_ >= waypoints_.size()) {
      return true;
    }

    const auto & wp = waypoints_[current_waypoint_index_];
    return std::hypot(wp.x - robot_x_, wp.y - robot_y_) < goal_tolerance_;
  }

  void publishStop()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_pub_->publish(cmd);
  }

  /**
   * Main control loop executed on a timer.
   * It waits for required sensor data, evaluates waypoint progress, and
   * publishes velocity commands based on obstacle proximity and heading.
   */
  void controlLoop()
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for odometry...");
      publishStop();
      return;
    }

    if (mode_ == "explore" && !have_map_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for map...");
      publishStop();
      return;
    }

    if (waypoints_.empty()) {
      if (mode_ == "explore" && have_map_) {
        generateCoverageWaypoints();
      } else {
        generateStraightLineWaypoints();
      }
    }

    while (current_waypoint_index_ < waypoints_.size() && waypointReached()) {
      ++current_waypoint_index_;
      RCLCPP_INFO(get_logger(), "Reached waypoint %zu / %zu.",
                  current_waypoint_index_, waypoints_.size());
    }

    if (current_waypoint_index_ >= waypoints_.size()) {
      publishStop();
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000, "Navigation/scanning complete.");
      return;
    }

    const auto & wp = waypoints_[current_waypoint_index_];
    const double dx = wp.x - robot_x_;
    const double dy = wp.y - robot_y_;
    const double distance_to_wp = std::hypot(dx, dy);
    const double desired_yaw = std::atan2(dy, dx);
    const double yaw_error = normalizeAngle(desired_yaw - robot_yaw_);

    geometry_msgs::msg::Twist cmd;

    if (have_scan_ && latest_front_distance_ < obstacle_stop_distance_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = latest_left_distance_ > latest_right_distance_
        ? max_angular_speed_
        : -max_angular_speed_;
      cmd_pub_->publish(cmd);

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Obstacle close: front=%.2f m. Turning to avoid.",
        latest_front_distance_);
      return;
    }

    const double angular_cmd = std::clamp(
      1.5 * yaw_error,
      -max_angular_speed_,
      max_angular_speed_);

    double speed_scale = 1.0;

    if (distance_to_wp < slow_down_distance_) {
      speed_scale = std::clamp(distance_to_wp / slow_down_distance_, 0.20, 1.0);
    }

    if (have_scan_ && latest_front_distance_ < obstacle_slow_distance_) {
      speed_scale *= std::clamp(latest_front_distance_ / obstacle_slow_distance_, 0.15, 1.0);
    }

    const double heading_scale = std::max(0.0, std::cos(yaw_error));

    cmd.linear.x = max_linear_speed_ * speed_scale * heading_scale;
    cmd.angular.z = angular_cmd;

    cmd_pub_->publish(cmd);
  }

  bool sim_mode_ = false;
  std::string isaac_namespace_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string cmd_vel_topic_;
  std::string mode_;

  double target_x_ = 2.0;
  double target_y_ = 0.0;
  double goal_tolerance_ = 0.30;
  double waypoint_spacing_ = 0.60;
  int coverage_spacing_cells_ = 8;

  double max_linear_speed_ = 0.30;
  double max_angular_speed_ = 0.80;
  double slow_down_distance_ = 0.75;
  double obstacle_stop_distance_ = 0.45;
  double obstacle_slow_distance_ = 0.80;

  int occupied_threshold_ = 65;
  bool unknown_is_blocked_ = true;

  double robot_x_ = 0.0;
  double robot_y_ = 0.0;
  double robot_yaw_ = 0.0;

  double latest_front_distance_ = std::numeric_limits<double>::infinity();
  double latest_left_distance_ = std::numeric_limits<double>::infinity();
  double latest_right_distance_ = std::numeric_limits<double>::infinity();

  bool have_scan_ = false;
  bool have_odom_ = false;
  bool have_map_ = false;
  bool generated_waypoints_ = false;

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  std::vector<Waypoint> waypoints_;
  size_t current_waypoint_index_ = 0;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationNode>());
  rclcpp::shutdown();
  return 0;
}
