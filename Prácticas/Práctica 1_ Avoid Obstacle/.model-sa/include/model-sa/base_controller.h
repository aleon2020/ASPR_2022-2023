/**
 * @file base_controller.h
 * @date 20/02/2023
 * @author Forocoches - URJC Robotics
*/

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

/**
 * @brief This class contains a node responsible for analyzing and managing the
 * behavior of a robot (Kobuki) based on the information taken from the laser scan
 * It is based in a probabilistic bi-dimentional finite status machine and is able 
 * to map the envioronment and avoid obstacles.  
*/
class BaseController : public rclcpp::Node
{

public:

  /**
   * @brief Posible sides to avoid an obstacle
  */
  enum Direction {
    LEFT = 1, 
    RIGHT = -1,
    NULLSIDE
  };

  /** @brief Posible status of the FSM */
  enum FSM_Status {
    FORWARD,      // Goes forward until an obstacle is found
    ROTATING,     // Rotates the kobuki until it is ready to sort it
    AVOIDING,     // Analyses and avoids the obstacle
    RELOCATING    // Tries to recover initial route
  };


  /**@brief Contructor. Node base_controller */
  BaseController();

  /** @brief Callback to analyse and response to /raw_scan (LaserScan) topics */
  void subscription_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /** @brief Publisher function to manage the FSM*/
  void timer_callback();
  /** @brief Publisher function to graphicate the Scanning algorithm*/
  void timer_callback_radar();
  
private:

  /**
   *  @brief Algorithm to split a set of floats (laser scans) into a set of sectors.
   *  @param data Set of sensor ranges
   *  @param sectors Number of splits to optimize (must be odd)
   *  @param range Maximum range for detecting the perturbance
   *  @return Set of booleans: 1 -> Sector with perturbance detected. 0 -> Sector free
   */ 
  std::vector<bool> sectorize(std::vector<float>& data, float sectors, float range);

  /**
   * @brief Algorithm to calculate the danger associated with a given sectorization.
   * @param status Set of data sectorized
   * @param Sensibility Influence of the sector danger to the result
   * @return 1 -> Danger. 0 -> Safe
  */
  bool obstacleAnalize(std::vector<bool>& status,int sensivity);

  /**
   * @brief Algorithm to determine the best side to avoid an obstacle
   * @param status Set of data sectorized
   * @return Direction in which the obstacle is positioned. (So avoid to the opposite)
  */
  Direction calculate_side_to_avoid_obstacle(std::vector<bool>& status);

  /**
   * @brief Translate a given sectorized set of data to a heat map (for graphicating)
   * @param frontal Sectorized set in the front
   * @param avoid Sectorized set at the sides
   * @note This function will modify the behavior of the intensities_ variable
  */
  void setIntensities(std::vector<bool>& frontal,std::vector<bool>& avoid);


  // -------- ROS Variables -------- 
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisherRadar_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timerRadar_;
  geometry_msgs::msg::Twist motion_;
  sensor_msgs::msg::LaserScan radar_;



  // -------- Kobuki Specific Variables -------- 
  bool isKobuki = false;
  std::vector<float> translate_kobuki_scan(std::vector<float> ranges);


  // -------- /scan_raw properties -------- 
  float angle_min;
  float angle_max;
  std::vector<float> ranges_;
  std::vector<float> intensities_;

  // -------- FSM Variables -------- 

  // Weight of the next probable status
  int load_forward;     
  int load_rotating;
  int load_relocating;
  int load_avoiding;

  FSM_Status status_ = FORWARD;           // Current status
  bool obstacleInFront_;                  // True if there is a obstacle at the front 
  bool obstacleInSide_;                   // True if there is a obstacle at the side 
  Direction sideToAvoidObstacle_ = RIGHT; // Direction where the robot should avoid
  Direction sideCompleted_;               // True if the avoid has been completed
  Direction lastSideRotated_ = NULLSIDE;  // Direction of the last avoid command 
  Direction AVOID_SIDE_BY_DEFAULT_;       // Avoid to a side by default (Deactivated by defect)
  rclcpp::Time end_time_;                 // Maximum time until the command should end
  int try_to_unlock_;                     // Maximum current weight punishable 
  int lock_remaining_;                    // Weight for punishing direction hesitation
  int conflict_penalty;                   // Weight of the punishment for status hesitation

  // -------- Machine Variables -------- 
  float velocity_;                // Velocity to move
  float recolocation_delay_;      // Patch to moderate recolocation aggresivity
  float rotation_smoothness_;     // Aggresivity of the avoidance. Smaller values = Softer avoidance
  float rotation_speed;           // Velocity to multiply when rotating
  float radious_;                 // Radious of the avoidance (shouldn't be modified. Change ↑ instead)
  int seconds_rotating_;          // Time to stay rotating before avoiding
  int seconds_recolocating_;      // Time to stay recolocating after an avoidance
  float velocity_delay_;          // Time to stay avoiding a obstacle
  int scan_resolution_ ;          // Resolution of the algorithm sectorization. Should be upper to 100
  float frontal_range_;           // Maximum range to stop detecting obstacles 
  float avoiding_range_;          // Maximum range to stop detecting obstacles (at the sides)
  int frontal_resolution_;        // Number of sectors to assign to the front of the robot (must be odd)
  int lateral_resolution_;        // Number of sectors to assign to the lateral of the robot (must be odd and > to the front)

};


