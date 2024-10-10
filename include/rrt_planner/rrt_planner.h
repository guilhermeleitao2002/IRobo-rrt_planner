
#ifndef _RTT_PLANNER_RTT_PLANNER_H
#define _RTT_PLANNER_RTT_PLANNER_H

#include <costmap_2d/costmap_2d.h>
#include <rrt_planner/collision_detector.h>
#include <rrt_planner/random_double_generator.h>
#include <rrt_planner/utils.h>
#include <vector>

namespace rrt_planner {

    class RRTPlanner {

        public:
            /**
             * @brief Constructor for the RRTPlanner object.
             * @param costmap A pointer to the costmap to use for planning.
             * @param params Struct with parameters to build the tree.
             */
            RRTPlanner(costmap_2d::Costmap2DROS *costmap, const rrt_params& params);

            /**
             * @brief   Plan a path using RRT
             * @return  True if found, false otherwise 
             */
            bool planPath();
            
            /**
             * @brief Sample random points
             * @return 2D random planar position (x, y)
             */
            double *sampleRandomPoint();

            /**
             * @brief Get the index of the nearest node around the new random point
             * @param point Random pointed sampled
             * @return The nearest node index
             */
            int getNearestNodeId(const double *point);

            /**
             * @brief Create new node
             * @param pos 2D coordinates of the new node
             * @param parent_node_id Id of the parent node of the new node
             */
            void createNewNode(const double* pos, int parent_node_id);

            /** 
             * @brief Connect the sampled random point to nearest tree node.
             * @param start 2D coordinates of nearest existing tree node to random point
             * @param goal  2D coordinates of sampled random point
             * @return 2D coordinates of potential new tree node
             */
            double *extendTree(const double* point_nearest, const double* point_rand);

            /*********************
             * Start and goal positions
             *********************/

            /**
             * @brief   Sets the start position for the planner
             * @param   start The start position
             */
            void setStart(double *start);

            /**
             * @brief   Sets the goal position for the planner
             * @param   start The goal position
             */
            void setGoal(double *goal);

            /**
             * @brief Get the RRT tree.
             */
            const std::vector<Node>& getTree();

            ~RRTPlanner() {};

            
            private:
                double start_[2], goal_[2];
                double rand_point_[2], candidate_point_[2];
                float map_width_, map_height_;
                double best_cost_;
                costmap_2d::Costmap2D* costmap_;
                std::vector<Node> nodes_;
                rrt_params params_;
                CollisionDetector collision_dect_;
                RandomDoubleGenerator random_double_x, random_double_y;

    };

};

#endif // _RTT_PLANNER_RTT_PLANNER_H


