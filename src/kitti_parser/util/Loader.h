#ifndef KITTI_PARSER_LOADER_H
#define KITTI_PARSER_LOADER_H


//class variant;

#include <string>
#include <iostream>
#include <vector>
#include <string>
#include <boost/variant.hpp>
#include "kitti_parser/util/Config.h"
#include "kitti_parser/types/stereo_t.h"
#include "kitti_parser/types/lidar_t.h"
#include "kitti_parser/types/gpsimu_t.h"


namespace kitti_parser {

    class Loader {

    public:

        // Default constructor
        Loader(Config* config);

        // Load all text files, and need paths
        void load_all(std::string path);


        // Fetches the latest measurement that should be processed
        typedef boost::variant<stereo_t*, lidar_t*, gpsimu_t*> message_types;
        message_types* fetch_latest();



    private:

        // Main config
        Config* config;

        // Main arrays of timestamps
        std::vector<long> time_stereo_gray;
        std::vector<long> time_stereo_color;
        std::vector<long> time_lidar_avg;
        std::vector<long> time_lidar_start;
        std::vector<long> time_lidar_end;


        // Main arrays of paths to data
        std::vector<std::string> path_stereo_gray_L;
        std::vector<std::string> path_stereo_gray_R;
        std::vector<std::string> path_stereo_color_L;
        std::vector<std::string> path_stereo_color_R;
        std::vector<std::string> path_lidar;


        // Master index values
        long curr_sg = LONG_MAX;
        long curr_sc = LONG_MAX;
        long curr_lidar = LONG_MAX;
        long curr_gps = LONG_MAX;
        size_t idx_sg = 0;
        size_t idx_sc = 0;
        size_t idx_lidar = 0;
        size_t idx_gps = 0;


        // Private functions to load each type
        void load_timestamps(std::string path_timestamp, std::vector<long>& time, int& ct);

        void load_stereo(std::string path_left, std::string path_right, std::vector<long>& time,
                         std::vector<std::string>& pathL, std::vector<std::string>& pathR);
        void load_lidar(std::string path_lidar,
                        std::vector<long>& time_avg, std::vector<long>& time_start,
                        std::vector<long>& time_end, std::vector<std::string>& pathB);


        // Fetch commands, constructs the actual datatype
        stereo_t* fetch_stereo(size_t idx, bool is_color);
        lidar_t* fetch_lidar(size_t idx);


    };

}


#endif //KITTI_PARSER_LOADER_H