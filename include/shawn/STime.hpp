#ifndef SHAWN_TIME_HPP
#define SHAWN_TIME_HPP

#include <chrono>
#include <vector>
#include <ctime>
#include <stdint.h>
#include <stdlib.h>
#include "SVector.hpp"
#include "SHandle.hpp"

namespace shawn {

class STime {
public:
    SVector shawn_vector;
    SHandle time_handle;

    std::vector<size_t> error_handle_IDs;

    std::chrono::time_point<std::chrono::steady_clock> simple_start_time_point;

    std::vector<std::chrono::time_point<std::chrono::steady_clock>> last_timestamps;
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> last_last_timestamps;
    std::vector<double> timestamps_duration_ms;
    std::vector<double> timestamps_till_now_ms;

    STime() {
        this->error_handle_IDs.clear();
        this->last_timestamps.clear();
        this->last_last_timestamps.clear();
        this->timestamps_duration_ms.clear();
        this->timestamps_till_now_ms.clear();
        this->setSimpleStartTimePoint();
    }

    inline std::chrono::time_point<std::chrono::steady_clock> getNowTimePoint() {
        return std::chrono::steady_clock::now();
    };

    bool pushTimestamp(handle p_time_handle, std::chrono::time_point<std::chrono::steady_clock> p_time_point = std::chrono::steady_clock::now()) {
        size_t tmp_ID;
        if (!this->time_handle.getID(p_time_handle, tmp_ID)) {
            return false;
        }
        if (this->last_timestamps.size() < (tmp_ID + 1)) {
            this->last_timestamps.resize(tmp_ID + 1);
            this->last_last_timestamps.resize(tmp_ID + 1);
        }
        this->last_last_timestamps[tmp_ID] = this->last_timestamps[tmp_ID];
        this->last_timestamps[tmp_ID] = p_time_point;
        return true;
    }

    void updateTimestampsDurationMs() {
        shawn_vector.unifySize(this->last_timestamps, this->timestamps_duration_ms);
        for (size_t i = 0; i < this->last_timestamps.size(); ++i) {
            this->timestamps_duration_ms[i] = 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(this->last_timestamps[i] - this->last_last_timestamps[i]).count();
        }
    }

    void updateTimestampsTillNowMs(std::chrono::time_point<std::chrono::steady_clock> p_now = std::chrono::steady_clock::now()) {
        shawn_vector.unifySize(this->last_timestamps, this->timestamps_till_now_ms);
        for (size_t i = 0; i < this->last_timestamps.size(); ++i) {
            this->timestamps_till_now_ms[i] = 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(p_now - this->last_timestamps[i]).count();
        }
    }

    bool checkTimestampsDuration(double p_min = -1, double p_max = -1) {
        this->updateTimestampsDurationMs();
        if (this->timestamps_duration_ms.empty()) {
            return false;
        }
        this->error_handle_IDs.clear();
        if (p_min > 0) {
            for (size_t i = 0; i < this->timestamps_duration_ms.size(); ++i) {
                if (this->timestamps_duration_ms[i] < p_min) {
                    this->error_handle_IDs.emplace_back(i);
                }
            }
        }
        if (p_max > 0) {
            for (size_t i = 0; i < this->timestamps_duration_ms.size(); ++i) {
                if (this->timestamps_duration_ms[i] > p_max) {
                    this->error_handle_IDs.emplace_back(i);
                }
            }
        }
        return this->error_handle_IDs.empty();
    }

    bool checkTimestampsTillNow(double p_min = -1, double p_max = -1, std::chrono::time_point<std::chrono::steady_clock> p_now = std::chrono::steady_clock::now()) {
        this->updateTimestampsTillNowMs(p_now);
        if (this->timestamps_till_now_ms.empty()) {
            return false;
        }
        this->error_handle_IDs.clear();
        if (p_min > 0) {
            for (size_t i = 0; i < this->timestamps_till_now_ms.size(); ++i) {
                if (this->timestamps_till_now_ms[i] < p_min) {
                    this->error_handle_IDs.emplace_back(i);
                }
            }
        }
        if (p_max > 0) {
            for (size_t i = 0; i < this->timestamps_till_now_ms.size(); ++i) {
                if (this->timestamps_till_now_ms[i] > p_max) {
                    this->error_handle_IDs.emplace_back(i);
                }
            }
        }
        return this->error_handle_IDs.empty();
    }

    double getTimestampsDurationMs(handle p_time_handle) {
        size_t tmp_ID;
        if (!this->time_handle.getID(p_time_handle, tmp_ID)) {
            return 0;
        }
        if (this->last_timestamps.size() < (tmp_ID + 1)) {
            return 0;
        }
        return 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(this->last_timestamps[tmp_ID] - this->last_last_timestamps[tmp_ID]).count();
    }

    double getTimestampsTillNowMs(handle p_time_handle, std::chrono::time_point<std::chrono::steady_clock> p_now = std::chrono::steady_clock::now()) {
        size_t tmp_ID;
        if (!this->time_handle.getID(p_time_handle, tmp_ID)) {
            return 0;
        }
        if (this->last_timestamps.size() < (tmp_ID + 1)) {
            return 0;
        }
        return 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(p_now - this->last_timestamps[tmp_ID]).count();
    }

    inline void setSimpleStartTimePoint() {
        this->simple_start_time_point = std::chrono::steady_clock::now();
    }

    inline double getSimpleDurationMs() {
        return 1000 * std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - this->simple_start_time_point).count();
    }


    bool checkSingleTimestampDuration(handle p_time_handle, double p_min = -1, double p_max = -1) {
        size_t tmp_ID;
        if (!this->time_handle.getID(p_time_handle, tmp_ID)) {
            return false;
        }
        if (this->last_timestamps.size() < (tmp_ID + 1)) {
            return false;
        }
        double_t duration = this->getTimestampsDurationMs(p_time_handle);
        if (p_min > 0) {
            if (duration < p_min) {
                return false;
            }
        }
        if (p_max > 0) {
            if (duration > p_max) {
                return false;
            }
        }
        return true;
    }

    bool checkSingleTimestampTillNow(handle p_time_handle, double p_min = -1, double p_max = -1, std::chrono::time_point<std::chrono::steady_clock> p_now = std::chrono::steady_clock::now()) {
        size_t tmp_ID;
        if (!this->time_handle.getID(p_time_handle, tmp_ID)) {
            return false;
        }
        if (this->last_timestamps.size() < (tmp_ID + 1)) {
            return false;
        }
        double_t till_now = this->getTimestampsTillNowMs(p_time_handle, p_now);
        if (p_min > 0) {
            if (till_now < p_min) {
                return false;
            }
        }
        if (p_max > 0) {
            if (till_now > p_max) {
                return false;
            }
        }
        return true;
    }
};

}

#endif //SHAWN_TIME_HPP