
#include <rrt_planner/rrt_planner.h>
#include <cstdlib> 

namespace rrt_planner {

    RRTPlanner::RRTPlanner(costmap_2d::Costmap2DROS *costmap, 
            const rrt_params& params) : params_(params), collision_dect_(costmap) {

        costmap_ = costmap->getCostmap();
        map_width_  = costmap_->getSizeInMetersX();
        map_height_ = costmap_->getSizeInMetersY();
        //map_width_  = 3;
        //map_height_ = 3;

        //ROS_INFO("map_width_: %f", map_width_);
        //ROS_INFO("map_height_: %f", map_height_);

        random_double_x.setRange(-map_width_, map_width_);
        random_double_y.setRange(-map_height_, map_height_);

        nodes_.reserve(params_.max_num_nodes);
    }

    bool RRTPlanner::planPath() {

        // clear everything before planning
        nodes_.clear();
        newBestNodeFound_ = 0;

        // Start Node
        createNewNode(start_, -1);
        best_pos_[0] = start_[0];
        best_pos_[1] = start_[1];
        best_node_id_ = 0;
        best_cost_ = computeDistance(start_, goal_);

        //ROS_WARN("start pos: %f, %f", start_[0], start_[1]);
        //ROS_WARN("goal pos: %f, %f", goal_[0], goal_[1]);
        //ROS_WARN("best_cost_initial: %f", best_cost_);

        double *p_rand, *p_new;
        Node nearest_node;
        int new_id;

        for (unsigned int k = 1; k <= params_.max_num_nodes; k++) {

            p_rand = sampleRandomPoint();
            //ROS_INFO("p_rand[0], p_rand[1]: %f, %f", p_rand[0], p_rand[1]);
            nearest_node = nodes_[getNearestNodeId(p_rand)];
            //ROS_INFO("nearest_node.pos[0], nearest_node.pos[1]: %f, %f", nearest_node.pos[0], nearest_node.pos[1]);
            p_new = extendTree(nearest_node.pos, p_rand); // new point and node candidate
            //ROS_INFO("Tree extended");

            if (!collision_dect_.obstacleBetween(nearest_node.pos, p_new)) {
                new_id = createNewNode(p_new, nearest_node.node_id);
                //ROS_INFO("New node created");
            } else {
                //ROS_INFO("Obstacle between");
                continue;
            }

            double dist = computeDistance(p_new, goal_);
            //ROS_WARN("dist: %f", dist);
            //ROS_WARN("best pos: %f, %f", p_new[0], p_new[1]);
            if (dist < best_cost_) {
                best_cost_ = dist;
                best_node_id_ = new_id;
                best_pos_[0] = p_new[0];
                best_pos_[1] = p_new[1];
                newBestNodeFound_ = 1;
                /* 
                */
                if (best_cost_ < params_.goal_tolerance) {

                    printf("dist: %f\n", dist);
                    printf("best_cost_: %f\n", best_cost_);
                    printf("computed distance %f\n", computeDistance(p_new, goal_));
                    printf("id: %d\n", best_node_id_);
                    printf("new_id %d\n", new_id);
                    printf("goal tolerance %f reached\n", params_.goal_tolerance);
                    return true;
                }
                //ROS_WARN("best_cost: %f", best_cost_);
                //ROS_WARN("best pos: %f, %f", p_new[0], p_new[1]);
                //ROS_WARN("best_node_id_: %d", best_node_id_);
            }

            if(k > params_.min_num_nodes) {
                
                if(computeDistance(p_new, goal_) <= params_.goal_tolerance) {
                    printf("best cost %f\n", best_cost_);
                    printf("computed distance %f\n", computeDistance(p_new, goal_));
                    printf("goal tolerance %f reached\n", params_.goal_tolerance);
                    return true;
                }
            }
        }

        return false;
    }

    // IMPLEMENTED THIS
    int RRTPlanner::getNearestNodeId(const double *point) {
        double shortestDistance;
        int nearestPoint;
        int started = 0;

        for (int i = 0; i < nodes_.size(); i++) {
            //ROS_WARN("BEFORE computeDistance");
            double distance = computeDistance(nodes_[i].pos, point);
            //ROS_WARN("AFTER computeDistance");
            if (!started) {
                shortestDistance = distance;
                nearestPoint = nodes_[i].node_id;
                started = 1;
                continue;
            }
            if (distance < shortestDistance) {
                shortestDistance = distance;
                nearestPoint = nodes_[i].node_id;
            }
        }
        //ROS_WARN("nearestPoint: %d", nearestPoint);
        return nearestPoint;
    }

    // IMPLEMENTED THIS
    int RRTPlanner::createNewNode(const double* pos, int parent_node_id) {

        Node new_node;

        new_node.node_id = nodes_.size(); // if node_id = -1, then node_id = 0
        new_node.pos[0] = pos[0];
        new_node.pos[1] = pos[1];
        new_node.parent_id = parent_node_id;

        //ROS_INFO("New node created at: %f, %f", new_node.pos[0], new_node.pos[1]);

        nodes_.emplace_back(new_node);

        return new_node.node_id;
    }

    // IMPLEMENTED THIS
    double* RRTPlanner::sampleRandomPoint() {
        double bias = 0.2;
        double random_for_bias = ((double) rand() / (RAND_MAX));
        
        double close_to_goal = 0.5;

        
        if (random_for_bias < bias) {
            rand_point_[0] = goal_[0];
            rand_point_[1] = goal_[1];
            return rand_point_;
        }
        else if (random_for_bias < close_to_goal) {
            random_double_x.setRange(start_[0], goal_[0]);
            random_double_y.setRange(start_[1], goal_[1]);

            rand_point_[0] = goal_[0] + random_double_x.generate();
            rand_point_[1] = goal_[1] + random_double_y.generate();
        }
        else {
            do {
                random_double_x.setRange(-map_width_, map_width_);
                random_double_y.setRange(-map_height_, map_height_);

                rand_point_[0] = random_double_x.generate();
                rand_point_[1] = random_double_y.generate();

                //ROS_WARN("rand_point_[0]: %f", rand_point_[0]);
                //ROS_WARN("rand_point_[1]: %f", rand_point_[1]);
            } while(collision_dect_.inFreeSpace(rand_point_));
        }
        

        return rand_point_;
    }

    // IMPLEMENTED THIS
    double* RRTPlanner::extendTree(const double* point_nearest, const double* point_rand) {
        double x = point_rand[0] - point_nearest[0];
        double y = point_rand[1] - point_nearest[1];

        double dist = computeDistance(point_nearest, point_rand);
        
        double normalized_vector_x = x / dist;
        double normalized_vector_y = y / dist;

        candidate_point_[0] = point_nearest[0] + normalized_vector_x * params_.step;
        candidate_point_[1] = point_nearest[1] + normalized_vector_y * params_.step;

        return candidate_point_;
    }

    const std::vector<Node>& RRTPlanner::getTree() {

        return nodes_;
    }

    void RRTPlanner::setStart(double *start) {

        start_[0] = start[0];
        start_[1] = start[1];
    }

    void RRTPlanner::setGoal(double *goal) {

        goal_[0] = goal[0];
        goal_[1] = goal[1];
    }

    int RRTPlanner::getNodeId(double *point) {
        for (int i = 0; i < nodes_.size(); i++) {
            if (nodes_[i].pos[0] == point[0] && nodes_[i].pos[1] == point[1]) {
                return nodes_[i].node_id;
            }
        }
        return -1;
    }

    int RRTPlanner::getBestNodeId() {
        return best_node_id_;
    }

    double RRTPlanner::getBestCost() {
        return best_cost_;
    }

    void RRTPlanner::setBestCost(double cost) {
        best_cost_ = cost;
    }

    double* RRTPlanner::getBestNodePos() {
        return best_pos_;
    }

    int RRTPlanner::newBestNodeFound() {
        return newBestNodeFound_;
    }

    void RRTPlanner::increaseObstacleCost() {
        collision_dect_.increaseObstacleCost();
    }

    void RRTPlanner::restoreObstacleCost() {
        collision_dect_.restoreObstacleCost();
    }

    void RRTPlanner::decreaseObstacleCost() {
        collision_dect_.decreaseObstacleCost();
    }
};