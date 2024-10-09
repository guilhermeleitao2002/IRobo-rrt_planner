
#ifndef _RTT_PLANNER_COLLISION_DETECTOR_H
#define _RTT_PLANNER_COLLISION_DETECTOR_H

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/cost_values.h>
#include <rrt_planner/utils.h>
#include <geometry_msgs/Point.h>
#include <math.h>
#include <vector>

namespace rrt_planner {

    class CollisionDetector {

        public:
            /**
             * @brief   Constructor for the CollisionDetector object
             * @param   costmap A pointer to the ROS wrapper of the costmap used for planning
             */ 
            explicit CollisionDetector(costmap_2d::Costmap2DROS* costmap);

            /**
             * @brief   Examine whether world_pos is in free space 
             * @param   world_pos 2D array of 2D coordinates (x, y) expressed in the world frame
             * @return  True if yes, false otherwise
             */ 
            bool inFreeSpace(const double* world_pos);

            /**
             * @brief   Examine whether there is an obstacle between point_a and point_b
             * @param   point_a array of 2D coordinates (x, y) expressed in the world frame
             * @param   point_b array of 2D coordinates (x, y) expressed in the world frame
             */ 
            bool obstacleBetween(const double* point_a, const double* point_b);

        private:
            costmap_2d::Costmap2D* costmap_;
            double robot_radius_;
            double resolution_, origin_x_, origin_y_;
    };


};

#endif // _RTT_PLANNER_COLLISION_DETECTOR_H