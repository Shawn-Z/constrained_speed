#ifndef SHAWN_PROPORTION_HPP
#define SHAWN_PROPORTION_HPP

#include <vector>
#include <cmath>
#include <stdlib.h>
#include <iostream>
#include <algorithm>

namespace shawn {
    class SProportion {
    public:
        SProportion() {
            this->steps.clear();
            this->proportions.clear();
            this->current_step = 0;
        }

        template<typename ... Types1>
        bool inputProportion(Types1 ... args) {
            size_t args_array[] = {args...};
            for (size_t i = 0; i < sizeof...(args); ++i) {
                if (args_array[i] <= 0) {
                    return false;
                }
                this->proportions.emplace_back(args_array[i]);
            }
            stepsGenerate();
            return true;
        }

        void stepsGenerate() {
            size_t total_size = 0;
            for (size_t i = 0; i < this->proportions.size(); ++i) {
                total_size += this->proportions[i];
            }
            while (this->steps.size() != total_size) {
                size_t position = std::distance(this->proportions.begin(), std::max_element(this->proportions.begin(), this->proportions.end()));
                this->steps.emplace_back(position);
                --this->proportions[position];
            }
        }

        size_t stepping() {
            if (this->current_step >= this->steps.size()) {
                this->current_step = 0;
            }
            size_t step_value = this->steps[this->current_step];
            ++this->current_step;
            return step_value;
        }

    private:
        std::vector<size_t> proportions;
        std::vector<size_t> steps;
        size_t current_step;
    };
}

#endif //SHAWN_PROPORTION_HPP