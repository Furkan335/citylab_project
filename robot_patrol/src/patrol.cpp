#include <cmath>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

/*
 * ============================================================================
 * PATROL CLASS - OVERVIEW
 * ============================================================================
 * This node makes the robot drive forward continuously, and steer away from
 * obstacles using its laser scanner (LIDAR).
 *
 * High level flow (runs continuously, twice per "cycle"):
 *
 *   1. laser_callback()  <- runs every time a new /fastbot_1/scan message
 *                            arrives. It reads the laser data and decides:
 *                              - is there an obstacle close ahead? (bool)
 *                              - if yes, which direction is safest to turn?
 *                            It does NOT publish anything itself - it just
 *                            updates two member variables that store the
 *                            current "decision".
 *
 *   2. control_loop()    <- runs on a fixed 10 Hz timer, independent of how
 *                            fast the laser data arrives. It reads the two
 *                            member variables set by laser_callback() and
 *                            publishes the actual velocity command.
 *
 * Splitting "sensing" (laser_callback) from "acting" (control_loop) means
 * the robot keeps publishing velocity commands at a steady rate even if the
 * laser topic publishes at an irregular rate.
 * ============================================================================
 */
class Patrol : public rclcpp::Node
{
public:
  Patrol()
  : Node("patrol_node")
  {
    // Subscribe to the laser scanner. Every time a new LaserScan message
    // arrives, ROS2 will automatically call laser_callback() for us.
    scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&Patrol::laser_callback, this, _1));

    // Publisher used to send velocity commands (linear + angular speed)
    // to the robot.
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    // A timer that fires every 100 ms (10 Hz) and calls control_loop().
    // This is what actually publishes cmd_vel messages, at a steady rate,
    // regardless of the laser's publish rate.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&Patrol::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "Patrol node started.");
  }

private:
  // --------------------------------------------------------------------
  // is_valid()
  // --------------------------------------------------------------------
  // A single laser range reading can be "bad" for several reasons:
  //   - NaN            -> sensor couldn't compute a value
  //   - Inf             -> no obstacle was detected within range_max
  //   - <= range_min    -> reading is below the sensor's minimum usable range
  //   - >= range_max    -> reading is at/beyond the sensor's maximum usable range
  // We must filter these out before using a reading in any distance
  // comparison, otherwise things like "find the maximum distance" would be
  // thrown off by a bogus Inf value.
  // --------------------------------------------------------------------
  bool is_valid(float range, float range_min, float range_max)
  {
    if (std::isnan(range) || std::isinf(range)) {
      return false;
    }
    if (range <= range_min || range >= range_max) {
      return false;
    }
    return true;
  }

  // --------------------------------------------------------------------
  // angle_from_front()
  // --------------------------------------------------------------------
  // WHY THIS FUNCTION EXISTS:
  // Each element in msg->ranges[] is a distance reading at a specific angle.
  // The angle for sample index "i" (going purely by the message definition)
  // is:  angle = angle_min + i * angle_increment
  //
  // For THIS robot, we empirically confirmed (by pointing it at a wall and
  // checking which index reported the shortest distance) that:
  //    - angle_min = 0.0
  //    - index 0 is the FRONT of the robot (straight ahead, +x axis)
  //    - the index increases COUNTERCLOCKWISE (towards the robot's LEFT)
  //      as you go around, matching ROS's standard right-hand convention
  //      (REP-103): positive angle = left turn, negative angle = right turn.
  //
  // This means the raw angle for an index near the end of the array (e.g.
  // index 199) is close to 360 degrees - but physically that is ALSO very
  // close to the front (just approached from the right side instead of the
  // left). Comparing raw angles like 359 degrees vs 0 degrees would
  // incorrectly treat them as "far apart" when physically they're neighbors.
  //
  // To fix this, we convert every angle into a "signed angle relative to
  // the front", wrapped into the range (-180, +180] degrees:
  //    -   0 deg   = straight ahead
  //    - +90 deg   = 90 degrees to the LEFT
  //    - -90 deg   = 90 degrees to the RIGHT
  //    - +/-180 deg = directly behind the robot
  //
  // With this convention, checking "is this within +/-90 degrees of the
  // front" becomes a simple fabs(angle) <= 90 check, and it automatically
  // handles the wrap-around at the start/end of the ranges[] array.
  // --------------------------------------------------------------------
  double angle_from_front(const sensor_msgs::msg::LaserScan::SharedPtr & msg, int i)
  {
    double angle = msg->angle_min + i * msg->angle_increment;

    // Wrap into (-pi, pi]
    while (angle > M_PI) {angle -= 2.0 * M_PI;}
    while (angle < -M_PI) {angle += 2.0 * M_PI;}

    return angle;
  }

  // --------------------------------------------------------------------
  // laser_callback()
  // --------------------------------------------------------------------
  // Runs every time a new laser scan arrives. Does two jobs:
  //
  //   STEP 1: Look only at a narrow +/-20 degree cone directly ahead.
  //           If the closest valid reading in that cone is under 35 cm,
  //           flag obstacle_detected_ = true.
  //
  //   STEP 2: If an obstacle was flagged, scan the WIDER +/-90 degree
  //           front area (the full front 180 degree field of view) and
  //           find the single valid reading with the GREATEST distance.
  //           That direction is the "most open" / safest direction to
  //           steer towards. We remember its angle in safest_angle_.
  //
  // Nothing is published here - this function only updates member
  // variables. Publishing happens separately in control_loop().
  // --------------------------------------------------------------------
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const int total_samples = static_cast<int>(msg->ranges.size());

    const double narrow_limit = 20.0 * M_PI / 180.0;  // +-20 deg, in radians
    const double front_limit = 90.0 * M_PI / 180.0;   // +-90 deg, in radians (front 180 FOV)

    // ---------------- STEP 1: narrow front-facing obstacle check ----------------
    obstacle_detected_ = false;
    float min_front_range = std::numeric_limits<float>::infinity();

    for (int i = 0; i < total_samples; ++i) {
      double angle = angle_from_front(msg, i);

      // Only consider rays inside the narrow +-20 degree cone in front.
      if (std::fabs(angle) <= narrow_limit) {
        float r = msg->ranges[i];
        if (is_valid(r, msg->range_min, msg->range_max) && r < min_front_range) {
          min_front_range = r;  // track the closest valid obstacle in the cone
        }
      }
    }

    // If the nearest thing directly ahead is closer than 35 cm, treat it as
    // "blocking the path".
    if (min_front_range < 0.35f) {
      obstacle_detected_ = true;
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "Front cone (+-20deg around index 0) min_range=%.3f m | obstacle_detected=%s",
      min_front_range, obstacle_detected_ ? "TRUE" : "false");

    // ---------------- STEP 2: find safest direction (only if blocked) ----------------
    safest_angle_ = 0.0;  // default: no obstacle -> "straight ahead" is safest

    if (obstacle_detected_) {
      float max_range = -1.0f;   // will hold the largest valid distance found
      int best_idx = 0;          // index of that reading (used only for logging)
      double best_angle = 0.0;   // angle of that reading -> this is what we act on

      for (int i = 0; i < total_samples; ++i) {
        double angle = angle_from_front(msg, i);

        // Only consider rays inside the front 180 degree FOV (+-90 deg).
        if (std::fabs(angle) <= front_limit) {
          float r = msg->ranges[i];
          if (is_valid(r, msg->range_min, msg->range_max) && r > max_range) {
            // This ray is the most "open" one seen so far -> remember it.
            max_range = r;
            best_idx = i;
            best_angle = angle;
          }
        }
      }

      safest_angle_ = best_angle;

      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 500,
        "Obstacle! Safest index=%d angle=%.1f deg (range=%.3f m) -> %s",
        best_idx, best_angle * 180.0 / M_PI, max_range,
        best_angle > 0 ? "turn LEFT" :
        best_angle < 0 ? "turn RIGHT" : "straight ahead (too close!)");
    }
  }

  // --------------------------------------------------------------------
  // control_loop()
  // --------------------------------------------------------------------
  // Runs on a steady 10 Hz timer (see constructor). Reads the decision
  // made by laser_callback() (obstacle_detected_ and safest_angle_) and
  // turns that into an actual velocity command that gets published.
  //
  //   - No obstacle  -> drive straight forward at normal speed.
  //   - Obstacle     -> slow down, and rotate towards whichever side
  //                     (left/right) had the most open space, based on
  //                     the SIGN of safest_angle_:
  //                        positive angle (left of front)  -> turn LEFT  (+angular.z)
  //                        negative angle (right of front) -> turn RIGHT (-angular.z)
  // --------------------------------------------------------------------
  void control_loop()
  {
    geometry_msgs::msg::Twist cmd_vel;

    if (!obstacle_detected_) {
      // Path is clear -> move forward, no turning.
      cmd_vel.linear.x = 0.1;
      cmd_vel.angular.z = 0.0;
    } else {
      // Obstacle detected -> slow down and turn towards the safest direction.
      cmd_vel.linear.x = 0.05;

      if (safest_angle_ > 0.0) {
        // Most open space is to the LEFT -> positive angular.z spins the
        // robot counterclockwise (left), per ROS's right-hand convention.
        cmd_vel.angular.z = 0.5;
      } else if (safest_angle_ < 0.0) {
        // Most open space is to the RIGHT -> negative angular.z spins the
        // robot clockwise (right).
        cmd_vel.angular.z = -0.5;
      } else {
        // Edge case: the "safest" direction is exactly straight ahead, but
        // we already know something is too close there. Pick a default
        // turn direction so the robot doesn't just sit there doing nothing.
        cmd_vel.angular.z = 0.5;
      }
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "Publishing cmd_vel -> linear.x=%.3f angular.z=%.3f",
      cmd_vel.linear.x, cmd_vel.angular.z);

    cmd_vel_publisher_->publish(cmd_vel);
  }

  // --------------------------------------------------------------------
  // Member variables
  // --------------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // These two variables are the "shared state" between laser_callback()
  // (which sets them) and control_loop() (which reads them). They persist
  // between calls because they are member variables of the class, not
  // local variables inside a function.
  bool obstacle_detected_ = false;  // true if something is too close ahead
  double safest_angle_ = 0.0;       // angle (rad) of the most open direction
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Patrol>();
  rclcpp::spin(node);  // keeps the node alive, processing callbacks/timers
  rclcpp::shutdown();
  return 0;
}