#include "model-sa/base_controller.h"

  BaseController::BaseController() : Node("base_controller")
  {
     // Setting parameters
     // TODO: Set this set of parameters as declare-param, get_param() + intialize
     // directly with this values at the launcher
    this->AVOID_SIDE_BY_DEFAULT_ = RIGHT;
    this->velocity_ = 0.2;
    this->rotation_smoothness_ = 2.3;
    this->recolocation_delay_ = 22;
    this->seconds_recolocating_= velocity_*recolocation_delay_;
    this->radious_ = 0.9;
    this->velocity_delay_ = 1.5;
    this->seconds_rotating_ = (M_PI*radious_)/(velocity_*velocity_delay_);
    this->scan_resolution_ = 131;
    this->frontal_range_ = 1;
    this->avoiding_range_ = 0.5;
    this->frontal_resolution_ = 51;
    this->lateral_resolution_ = 61;
    this->isKobuki = true;
    this->rotation_speed = 3;

    // Note this parameters should be initialized here 
    this->load_forward = 130;
    this->load_rotating = 0;
    this->load_relocating = 0;
    this->load_avoiding = 0;
    this->conflict_penalty = 120;
    
    this->intensities_ = std::vector<float>(scan_resolution_,0);
    this->ranges_ = std::vector<float>(scan_resolution_,1);

    // Building Subscriber to the Laser
    subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan_raw",10,
        std::bind(&BaseController::subscription_callback, this, std::placeholders::_1));

    // Building Publisher to the velocity node
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    timer_ = create_wall_timer(
        10ms, std::bind(&BaseController::timer_callback, this));

    // Building Publisher to graphicate to rviz
    publisherRadar_ = create_publisher<sensor_msgs::msg::LaserScan>("my_radar",10);
    timerRadar_ = create_wall_timer(
        10ms, std::bind(&BaseController::timer_callback_radar, this));
  }

  void BaseController::subscription_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Get the message
    auto scan = msg.get();

    // Save the properties
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    std::vector<float> ranges;

    // If is kobuki patch the laser (by default would be analysed as Tiago)
    if (isKobuki){
      ranges = translate_kobuki_scan(scan->ranges);
    }else{
      ranges = scan->ranges;
    }

    // Sectorize the scan in function of the danger at the front 
    auto frontal_scan = sectorize(ranges,scan_resolution_,frontal_range_);
    obstacleInFront_ = obstacleAnalize(frontal_scan,frontal_resolution_);

    // Sectorize the scan in function of the danger at the sides 
    auto avoiding_scan = sectorize(ranges,scan_resolution_,avoiding_range_);
    obstacleInSide_ = obstacleAnalize(avoiding_scan,lateral_resolution_);

    // Stablish best side to avoid (Inestable response if obstacle not detected) 
    sideToAvoidObstacle_ = calculate_side_to_avoid_obstacle(avoiding_scan);

    // Reformat the scan to graphicable values 
    setIntensities(frontal_scan, avoiding_scan);
  }

  void BaseController::timer_callback_radar(){

      // Prepare the properties of the graphication
      radar_.angle_min = angle_min;
      radar_.angle_max = angle_max;
      radar_.angle_increment = (-angle_min + angle_max)/scan_resolution_;
      radar_.header.frame_id = "base_link"; // Important: This is the reference to take in Rviz2
      radar_.time_increment = 0.010;
      radar_.range_min = 0; 
      radar_.range_max = 2; 
      radar_.ranges = this->ranges_;
      radar_.intensities = this->intensities_; 

      // Publish
      publisherRadar_->publish(radar_);
  }

  void BaseController::timer_callback()
  {

    // Simple logger of the FSM
    // TODO: Replace this logger with a debug node (if reliable)
    RCLCPP_INFO(get_logger(),"[%d,%d,%d,%d] Locker: %d. %s [%s,%s]",
      load_forward,     // Weight of the status 1
      load_rotating,    // Weight of the status 2
      load_avoiding,    // Weight of the status 3
      load_relocating,  // Weight of the status 4
      lock_remaining_,  // Punishment aplplied for direction hesitation (at the time)
      (sideToAvoidObstacle_ == LEFT) ? "LEFT" : "RIGHT",   // Next best side to avoid
      (obstacleInFront_) ? "FRONT" : "0",  // FRONT if danger at the front
      (obstacleInSide_) ? "AVOID" : "0"    // AVOID if danger at the side
    );

    // Select the next status in function of each weight
    if (load_forward > conflict_penalty){
      status_ = FORWARD;
    }
    if (load_avoiding > conflict_penalty){
      status_ = AVOIDING;
    }
    if (load_relocating > conflict_penalty){
      status_ = RELOCATING;
    }
    if (load_rotating > conflict_penalty){
      status_ = ROTATING;
    }

    switch (status_)
    {

    case FORWARD: 

      // Reset the variables
      try_to_unlock_ = 0;
      lock_remaining_ = 0;
      lastSideRotated_ = NULLSIDE; // Forget the last side used (optimization)

      // Move forward
      motion_.linear.set__x(velocity_);
      motion_.angular.set__z(0);

      // Stop when finding an obstacle
      if (obstacleInFront_){

          // Add the weights to change status
          load_rotating++;
          this->load_forward = 0;
          this->load_relocating = 0;
          this->load_avoiding = 0;
      }
      break;

    case ROTATING:

      // Set velocity to 0
      motion_.linear.set__x(0);

      // If you are switching between directions start growing the punishment effect 
      if (lastSideRotated_ != sideToAvoidObstacle_ && lastSideRotated_ != NULLSIDE){
        try_to_unlock_++;
        lock_remaining_ = try_to_unlock_;
      }

      // If you are not locked (because of punishment), turn to the side scanned and save the direction 
      if (lock_remaining_ == 0){

        if (sideToAvoidObstacle_ == LEFT)
        {
          motion_.angular.set__z(velocity_*rotation_speed);
          lastSideRotated_ = sideToAvoidObstacle_;
        }
        else if (sideToAvoidObstacle_ == RIGHT)
        {
          motion_.angular.set__z(-velocity_*rotation_speed);
          lastSideRotated_ = sideToAvoidObstacle_;
        }

      // If you are locked then repeat the last direction (stop switching)
      } else {
        if (lastSideRotated_ == LEFT)
        {
          motion_.angular.set__z(velocity_*rotation_speed);
        }
        else if (lastSideRotated_ == RIGHT)
        {
          motion_.angular.set__z(-velocity_*rotation_speed);
        }
        // Decrease punishment variable
        lock_remaining_--;
      }

      // If you avoided the obstacle, change status (add the weights)
      if (!obstacleInSide_) {

        // Start a timer for the next status
        end_time_ = rclcpp::Time(this->now() + std::chrono::seconds(seconds_rotating_));
        this->load_avoiding++;
        this->load_rotating = 0;
        this->load_forward = 0;
        this->load_relocating = 0;

      }

      // If you are in a narrow passage make a slow step 
      // [WARNING] Mode Disabled. Working nice in simulator, although in reality
      // narrow passages are dangerous 
      //if (obstacleInSide_ && !obstacleInFront_){
      //   motion_.angular.set__z(0);
      //   motion_.linear.set__x(velocity_/2);
      // }

      break;

    case AVOIDING:

      // Start moving forward 
      motion_.linear.set__x(velocity_);

      // If you find and obstacle, add weights to the turning status
      if (obstacleInFront_ && obstacleInSide_) {
        
        // Reset other variables (optimization)
        try_to_unlock_ = 0;
        lock_remaining_ = 0;
        lastSideRotated_ = NULLSIDE;

        // Add weights
        load_rotating++;
        this->load_avoiding = 0;
        this->load_forward = 0;
        this->load_relocating = 0;
      }

      // If you turned to one side, then avoid to the opposite
      if (sideToAvoidObstacle_ == LEFT){
        motion_.angular.set__z(-velocity_*rotation_smoothness_);
      }else{
        motion_.angular.set__z(velocity_*rotation_smoothness_);
      }

      // If the avoidance finish
      if (this->now() > end_time_){

        // Reset variables (optimization)
        try_to_unlock_ = 0;
        lock_remaining_ = 0;
        lastSideRotated_ = NULLSIDE;

        // Start a timer (for next status)
        end_time_ = rclcpp::Time(this->now() + std::chrono::seconds(seconds_recolocating_));
        sideCompleted_ = sideToAvoidObstacle_;

        // Reset the status and jump directly to the next status (without weight)
        status_ = RELOCATING;
        this->load_relocating = 0;
        this->load_forward = 0;
        this->load_rotating = 0;
        this->load_avoiding = 0;

      }
      break;

    case RELOCATING:

      // Stop moving
      motion_.linear.set__x(0);

      // Relocate to the opposite side of the avoidance
      if (sideCompleted_ == LEFT){
        motion_.angular.set__z(velocity_*rotation_speed);
      }else{
        motion_.angular.set__z(-velocity_*rotation_speed);
      }

      // If you finished relocating
      if (this->now() > end_time_){

        // Restart the FSM
        status_ = FORWARD;
        this->load_forward = 0;
        this->load_relocating = 0;
        this->load_rotating = 0;
        this->load_avoiding = 0;
      }
      break;
    }

    // Publish the result of the FSM 
    publisher_->publish(motion_);

  }

  std::vector<bool> BaseController::sectorize(std::vector<float>& data, float sectors, float range){
    
    // How does this work:
    // For sectorizing a set of data only is needed to calculate the mean of the
    // sum of the measurements taken from the laser. This will be repeated <resolution>
    // times and will be comparted with the range variable to determine wheter the sector
    // has a perturbance or not.
    // This will return for example: [0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0 ...]xN 
    // (note how the object are represented with 1's)

    std::vector<bool> vec;                      // Vector with sector in range status
    int valuesInVector = data.size()/sectors;   // Number of data measurements in each sector
    float sum;                                  // Sum of all measurements (per sector)
    for (int sector = 0; sector < sectors; sector++){
      sum = 0;

      // Calculate the mean of the sector
      for (int value = 0; value < valuesInVector; value++){
        sum += data[value + sector*valuesInVector];
      }
      sum = sum/valuesInVector;

      // Apend status of the sector
      vec.push_back(sum <= range && sum >= 0.2);

      // [Note] Use this logger to directly retreive the sectorization of each measurement
      // RCLCPP_INFO(get_logger(), (sum <= range) ? "1" : "0");    
    }    
    return vec;
  }

  bool BaseController::obstacleAnalize(std::vector<bool>& status, int sensivity){

    // This will look search <sensivity/2> measurements next to
    // the center of the vector. It will mark the sectors specified 
    // according to the sensitivity of the system as those with greater
    // relevance 
    // For example:
    // vector.size() = 9, sensivity = 3
    // - - - X X X - - - 
    int begin = (status.size() - sensivity)/2;
    int end = (status.size() + sensivity)/2 + 1;

    for (int i = begin; i <= end; i++){
      if (status[i-1]){
        return true;
      }
    }

    return false;

  }

  BaseController::Direction BaseController::calculate_side_to_avoid_obstacle(std::vector<bool>& status){

    // Analyse the given sectorization and iterate until you find the first free space
    // going from the center (front) to the extremes of the vector
    // By testing, it returns the best side to avoid in >80% of the cases
    bool obstacle_at_right = true;
    bool obstacle_at_left = true;

    for (int i = status.size()/2; i >= 1; i--){

      if (!status[i-1]){
        obstacle_at_right = false;
      }

      if (!status[status.size() - i])
      {
        obstacle_at_left = false;
      }

      if (obstacle_at_left && !obstacle_at_right){
        return RIGHT;
      }
      if (!obstacle_at_left && obstacle_at_right) {
        return LEFT;
      }

    }

    return sideToAvoidObstacle_; 
  }

  void BaseController::setIntensities(std::vector<bool>& frontal,std::vector<bool>& avoid){

    // Asignes 0,1,2 to the satatuses of AVOID,FRONTAL,FREE (for graphication)
    for (int i = 0; i < scan_resolution_; i++)
    {
      if (avoid[i])
      {
        intensities_[i] = 2;
      }
      else if (frontal[i])
      {
        intensities_[i] = 1;
      }
      else
      {
        intensities_[i] = 0;
      }
    }
  }

  std::vector<float> BaseController::translate_kobuki_scan(std::vector<float> ranges){

      // Remap the laser as the same order of a Tiago laser. This means
      // starting at -1.91rad to 1.91 rad instead of 0rad to 6.4rad
      int center = ranges.size()/2;
      std::vector<float> newRanges;      

      for (int i = 0; i < center; i++){

        newRanges.push_back(ranges[center+i]);

      }

      for (int i = 0; i < center; i++){
        newRanges.push_back(ranges[i]);
      }

      return newRanges;

  }
