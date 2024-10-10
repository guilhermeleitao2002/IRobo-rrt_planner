
#include <rrt_planner/rrt_planner_ros.h>
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlannerROS, nav_core::BaseGlobalPlanner)

namespace rrt_planner {

    RRTPlannerROS::RRTPlannerROS(): 
        costmap_(NULL), planner_(), initialized_(false) {}

    RRTPlannerROS::RRTPlannerROS(std::string name, 
        costmap_2d::Costmap2DROS *costmap): costmap_(NULL), 
                              planner_(), initialized_(false) {
            
            initialize(name, costmap);
    }

    RRTPlannerROS::RRTPlannerROS(std::string name, 
        costmap_2d::Costmap2DROS *costmap, std::string global_frame): 
                      costmap_(NULL), planner_(), initialized_(false) {
            
            initialize(name, costmap, global_frame);
    }

    void RRTPlannerROS::initialize(std::string name, 
            costmap_2d::Costmap2DROS *costmap, std::string global_frame) {
        
        if(!initialized_) {
            
            costmap_ = costmap->getCostmap();
            global_frame_ = global_frame;

            ros::NodeHandle nh("~/" + name);         
            nh.param("goal_tolerance", params_.goal_tolerance, 0.15);
            nh.param("rrt/step", params_.step, 0.15);
            nh.param("rrt/min_num_nodes", params_.min_num_nodes, 5000);
            nh.param("rrt/max_num_nodes", params_.max_num_nodes, 30000);

            plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);

            planner_ = std::shared_ptr<RRTPlanner>(new RRTPlanner(costmap, params_));
            initialized_ = true;
        }
        else {

            ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
        }    
    }
    
    void RRTPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS *costmap) {

        initialize(name, costmap, costmap->getGlobalFrameID());
    }

    bool RRTPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, 
                                    const geometry_msgs::PoseStamped& goal, 
                                        std::vector<geometry_msgs::PoseStamped>& plan) {

      if(!initialized_){
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
      }

      // clear the plan, just in case
      plan.clear();

      if(start.header.frame_id != global_frame_){
        ROS_ERROR("The start pose passed to this planner must be in the %s frame. It is instead in the %s frame.", global_frame_.c_str(), start.header.frame_id.c_str());
        return false;
      }

      if(goal.header.frame_id != global_frame_){
        ROS_ERROR("The goal pose passed to this planner must be in the %s frame. It is instead in the %s frame.", global_frame_.c_str(), goal.header.frame_id.c_str());
        return false;
      }

      double world_start[2];
      world_start[0] = start.pose.position.x;
      world_start[1] = start.pose.position.y;

      unsigned int map_x, map_y;
      if(!costmap_->worldToMap(world_start[0], world_start[1], map_x, map_y)) {

        ROS_WARN_THROTTLE(1.0, "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
      }

      double world_goal[2];
      world_goal[0] = goal.pose.position.x;
      world_goal[1] = goal.pose.position.y;

      if(!costmap_->worldToMap(world_goal[0], world_goal[1], map_x, map_y)) {

          ROS_WARN_THROTTLE(1.0, "The goal sent to the planner is off the global costmap. Planning will always fail to this goal.");
          return false;
      }

      ROS_INFO("[RRTPlanner] Current Position: (%.2lf, %.2lf)", world_start[0], world_start[1]);
      ROS_INFO("[RRTPlanner] GOAL Position: (%.2lf, %.2lf)", world_goal[0], world_goal[1]);

      planner_->setStart(world_start);
      planner_->setGoal(world_goal);

      if( planner_->planPath() ){

        plan_time_ = ros::Time::now();

        rrt_tree_ = planner_->getTree();
        current_id_ = rrt_tree_.size() - 1; // Add last vertex (closest to goal)

        pose_stamped_.header.stamp = plan_time_;
        pose_stamped_.header.frame_id = global_frame_;

        pose_stamped_.pose.orientation = goal.pose.orientation; // Set goal orientation for last node
        
        // Work our way back to start waypoint, building plan
        while (current_id_ != 0) {
          
          // Retrieve pose of current ID
          pose_stamped_.pose.position.x = rrt_tree_[current_id_].pos[0];
          pose_stamped_.pose.position.y = rrt_tree_[current_id_].pos[1];
          pose_stamped_.pose.position.z = 0.;
          plan.push_back(pose_stamped_);           // Add pose to plan

          prev_id_ = current_id_; // Identify next vertex in path (parent node), store previous ID
          current_id_ = rrt_tree_[current_id_].parent_id;

          // Set orientation for next iteration
          dy_ = rrt_tree_[prev_id_].pos[1] - rrt_tree_[current_id_].pos[1];
          dx_ = rrt_tree_[prev_id_].pos[0] - rrt_tree_[current_id_].pos[0];

          yaw_ = atan2(dy_, dx_);  // Get yaw from atan2 using current point and previous point
          quat_tf_.setRPY(0., 0., yaw_); // Convert RPY to quat
          quat_msg_ = tf2::toMsg(quat_tf_); // Convert Quat TF to msg
          pose_stamped_.pose.orientation = quat_msg_; // set orientation
        }

        // Add start waypoint
        pose_stamped_.pose.position.x = rrt_tree_[0].pos[0];
        pose_stamped_.pose.position.y = rrt_tree_[0].pos[1];
        pose_stamped_.pose.position.z = 0.;
        pose_stamped_.pose.orientation = start.pose.orientation;
        plan.push_back(pose_stamped_);

        // Reverse so that initial waypoint is first and goal is last
        std::reverse(plan.begin(), plan.end());
        publishPlan(plan);

        return true;

      } else {

        ROS_WARN("[RRTPlanner] Failed to find a path.");
        return false;
      }
  }

  void RRTPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {

    if(!initialized_){
      ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
      return;
    }

    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(path.empty()) {
      //still set a valid frame so visualization won't hit transform issues
    	gui_path.header.frame_id = global_frame_;
      gui_path.header.stamp = ros::Time::now();
    } else { 
      gui_path.header.frame_id = path[0].header.frame_id;
      gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i = 0; i < path.size(); i++) {
      gui_path.poses[i] = path[i];
    }
    
    plan_pub_.publish(gui_path);
  }

};   