#ifndef SHAWN_LOG_HPP
#define SHAWN_LOG_HPP

#include <vector>
#include <ctime>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <glog/logging.h>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <sys/stat.h>

namespace shawn {
    class SLog {
    public:
        void init(std::string p_name, std::string p_folder = "log_dir", google::LogSeverity p_stderr_severity = google::ERROR) {
            google::InitGoogleLogging(p_name.c_str());
            std::string folder = getenv("HOME");
            folder += "/";
            folder += p_folder;
            if (access(folder.c_str(), 06) < 0) {
                while (mkdir(folder.c_str(), 0777) < 0) {
                    std::cerr<<"glog -- create folder failed"<<std::endl;
                }
            }
            FLAGS_log_dir = folder;
            google::SetStderrLogging(p_stderr_severity);
        }

        void logUint8Array(char *p_array, size_t p_array_size, google::LogSeverity p_log_severity) {
            std::vector<uint8_t> log_data(p_array, p_array + p_array_size);
            this->logUint8Vector(log_data, p_log_severity);
        }

        void logUint8Vector(std::vector<uint8_t> &p_log_data, google::LogSeverity p_log_severity) {
            if (p_log_severity == google::INFO) {
                auto&& log = COMPACT_GOOGLE_LOG_INFO;
                log.stream() << "string array data: 0x";
                for(int cell: p_log_data) {
                    log.stream() << " " << std::setfill('0') << std::setw(2) << std::hex << cell;
                }
            }
            if (p_log_severity == google::WARNING) {
                auto&& log = COMPACT_GOOGLE_LOG_WARNING;
                log.stream() << "string array data: 0x";
                for(int cell: p_log_data) {
                    log.stream() << " " << std::setfill('0') << std::setw(2) << std::hex << cell;
                }
            }
            if (p_log_severity == google::ERROR) {
                auto&& log = COMPACT_GOOGLE_LOG_ERROR;
                log.stream() << "string array data: 0x";
                for(int cell: p_log_data) {
                    log.stream() << " " << std::setfill('0') << std::setw(2) << std::hex << cell;
                }
            }
            if (p_log_severity == google::FATAL) {
                auto&& log = COMPACT_GOOGLE_LOG_FATAL;
                log.stream() << "string array data: 0x";
                for(int cell: p_log_data) {
                    log.stream() << " " << std::setfill('0') << std::setw(2) << std::hex << cell;
                }
            }
        }
    };
}

#endif //SHAWN_LOG_HPP