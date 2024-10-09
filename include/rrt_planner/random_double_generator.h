#ifndef _RTT_PLANNER_RANDOM_DOUBLE_GENERATOR_H
#define _RTT_PLANNER_RANDOM_DOUBLE_GENERATOR_H

#include <random>
#include <cfloat>

namespace rrt_planner {

    class RandomDoubleGenerator {

        public:
            RandomDoubleGenerator() = default;
            void setRange(const double& min, const double& max);
            double generate();

        private:
            std::random_device rd_;
            double min_value_{-1.0};
            double max_value_{1.0};
    };
};

#endif // _RTT_PLANNER_RANDOM_DOUBLE_GENERATOR_H