
#ifndef _RTT_PLANNER_UTILS_H
#define _RTT_PLANNER_UTILS_H

namespace rrt_planner {

    struct rrt_params {
        int min_num_nodes;
        int max_num_nodes;
        double goal_tolerance;
        double step;
    };

    struct Node {
        double pos[2]; // 2D coordinates (x, y)
        int node_id;
        int parent_id;
        float cost_to_go{0.0};

        Node() {}

        Node(const double *pos_, int node_index, int parent_index) : 
                pos{pos_[0], pos_[1]}, node_id(node_index), parent_id(parent_index) {}

        bool operator ==(const Node& node) { return node_id == node.node_id; }

        bool operator !=(const Node& node) { return !(node_id == node.node_id); }
    };

    inline double computeDistance(const double *x, const double *y) {
        return std::hypot((y[1] - x[1]), (y[0] - x[0])); 
    }

};

#endif // _RTT_PLANNER_UTILS_H