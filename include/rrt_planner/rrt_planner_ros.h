
#ifndef _RTT_PLANNER_RTT_PLANNER_ROS_H
#define _RTT_PLANNER_RTT_PLANNER_ROS_H

#include <ros/ros.h>
#include <rrt_planner/rrt_planner.h>
#include <rrt_planner/utils.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Path.h>
#include <vector>

namespace rrt_planner {

    /**
     * @class RRTPlannerROS
     * @brief Provides a ROS wrapper for the RRT planner.
     */
    class RRTPlannerROS : public nav_core::BaseGlobalPlanner {

        public:
            /**
             * @brief   Default constructor for the RRTPlannerROS object.
             */
            RRTPlannerROS();

            /**
             * @brief   Constructor for the RRTPlannerROS object
             * @param   name The name of this planner
             * @param   costmap A pointer to the ROS wrapper of the costmap to use for planning
             */    
            RRTPlannerROS(std::string name, costmap_2d::Costmap2DROS *costmap);

            /**
             * @brief   Constructor for the RRTPlannerROS object
             * @param   name The name of this planner
             * @param   costmap A pointer to the costmap to use for planning
             * @param   global_frame The global frame of the costmap
             */    
            RRTPlannerROS(std::string name, costmap_2d::Costmap2DROS *costmap, std::string global_frame);
            
            /**
             * @brief   Initialization function for the RRTPlannerROS object 
             * @param   name The name of this planner
             * @param   costmap A pointer to the ROS wrapper of the costmap to use for planning
             */
            void initialize(std::string name, costmap_2d::Costmap2DROS *costmap);

            /**
             * @brief   Initialization function for the RRTPlannerROS object 
             * @param   name The name of this planner
             * @param   costmap A pointer to the costmap to use for planning
             * @param   global_frame The global frame of the costmap
             */
            void initialize(std::string name, costmap_2d::Costmap2DROS *costmap, std::string global_frame);

            /**
             * @brief   Given a goal pose in the world, compute a plan
             * @param   start The start pose
             * @param   goal The goal pose
             * @param   plan The plan ... filled by the planner
             * @return  True if a valid plan was found, false otherwise 
             */
            bool makePlan(const geometry_msgs::PoseStamped& start,
                                const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan);


            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);

            ~RRTPlannerROS() {};


        protected:
            /**
             * @brief Store a copy of the current costmap in \a costmap. Called by makePlan.
             */
            costmap_2d::Costmap2D *costmap_;
            std::shared_ptr<RRTPlanner> planner_;
            ros::Publisher plan_pub_;
            bool initialized_;


        private:
            std::vector<Node> rrt_tree_;
            rrt_params params_;
            std::string global_frame_;

            // Path variables
            int current_id_, prev_id_;
            double dy_, dx_, yaw_;
            ros::Time plan_time_;
            tf2::Quaternion quat_tf_;
            geometry_msgs::Quaternion quat_msg_;
            geometry_msgs::PoseStamped pose_stamped_;

    };

};

#endif // _RTT_PLANNER_RTT_PLANNER_ROS_H
