#include <rrt_planner/collision_detector.h>

namespace rrt_planner {

    CollisionDetector::CollisionDetector(costmap_2d::Costmap2DROS* costmap) {

        costmap_ = costmap->getCostmap();

        resolution_ = costmap_->getResolution();
        origin_x_ = costmap_->getOriginX();
        origin_y_ = costmap_->getOriginY();
        robot_radius_ = 0.210; // TODO: Get this from the robot model // 0.205

    }

    bool CollisionDetector::inFreeSpace(const double* world_pos) {

        unsigned int mx_center, my_center;
        // Convert world coordinates to map coordinates
        if (!costmap_->worldToMap(world_pos[0], world_pos[1], mx_center, my_center)) {
            // Point is outside the map bounds
            return false;
        }

        // Calculate the number of cells that cover the robot's radius
        int cell_radius = static_cast<int>(ceil(robot_radius_ / resolution_));

        // Iterate over the square that bounds the circle
        for (int dx = -cell_radius; dx <= cell_radius; dx++) {
            for (int dy = -cell_radius; dy <= cell_radius; dy++) {

                int mx = mx_center + dx;
                int my = my_center + dy;

                // Check if the cell is within the map bounds
                if (mx < 0 || mx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
                    my < 0 || my >= static_cast<int>(costmap_->getSizeInCellsY())) {
                    // Out of bounds, treat as occupied
                    return false;
                }

                // Calculate the distance from the center cell to this cell
                double x = (mx - mx_center) * resolution_;
                double y = (my - my_center) * resolution_;
                double distance = sqrt(x * x + y * y);

                if (distance <= robot_radius_) {
                    // Check the cost at this cell
                    unsigned char cost = costmap_->getCost(mx, my);
                    if (cost >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                        // Collision detected within robot's radius
                        return false;
                    }
                }
            }
        }

        // No collision detected within robot's radius
        return true;
    }

    bool CollisionDetector::obstacleBetween(const double* point_a, const double* point_b) {

        double dist = computeDistance(point_a, point_b);

        if (dist < resolution_) {
            return ( !inFreeSpace(point_b) ) ? true : false;

        } else {
            
            int num_steps = static_cast<int>(floor(dist/resolution_));

            double point_i[2];
            for (int n = 1; n <= num_steps; n++) {

                point_i[0] = point_a[0] + n * (point_b[0] - point_a[0]) / num_steps;
                point_i[1] = point_a[1] + n * (point_b[1] - point_a[1]) / num_steps;

                if ( !inFreeSpace(point_i) ) return true;
            }
            
            return false;
        }

    }

};
