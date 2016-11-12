#ifndef KITTI_PARSER_CONFIG_H
#define KITTI_PARSER_CONFIG_H

#include <string>
#include <iostream>


namespace kitti_parser {

    class Config {

    public:

        // Core paths
        std::string path_data;
        std::string path_calib_cc;
        std::string path_calib_iv;
        std::string path_calib_vc;

        // Sensors enabled / disabled
        bool has_lidar = false;
        bool has_gpsimu = false;
        bool has_stereo_gray = false;
        bool has_stereo_color = false;


        // Config information
        bool has_calib_cc = false;
        bool has_calib_iv = false;
        bool has_calib_vc = false;


        // TODO: Store the config data here



    };
}

#endif //KITTI_PARSER_CONFIG_H