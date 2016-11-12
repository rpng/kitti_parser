#ifndef KITTI_PARSER_PARSER_H
#define KITTI_PARSER_PARSER_H


#include <string>
#include <iostream>
#include <functional>
#include "kitti_parser/util/Config.h"
#include "kitti_parser/util/Loader.h"
#include "kitti_parser/types/stereo_t.h"
#include "kitti_parser/types/lidar_t.h"
#include "kitti_parser/types/gpsimu_t.h"



namespace kitti_parser {

    class Parser {

    public:

        // Default constructor
        Parser(std::string data_path);

        // Returns the current config
        Config getConfig();

        // Register callback functions
        void register_callback_stereo_gray(std::function<void(Config*,long, stereo_t*)> callback);
        void register_callback_stereo_color(std::function<void(Config*,long, stereo_t*)> callback);
        void register_callback_lidar(std::function<void(Config*,long, lidar_t*)> callback);
        void register_callback_gpsimu(std::function<void(Config*,long, gpsimu_t*)> callback);

        // Main run function, will call callbacks
        void run(double time_multi);


    private:

        // Main config
        Config config;

        // Loader with all the data
        Loader* loader;

        // List of callback functions to call
        std::function<void(Config*,long, stereo_t*)> callback_stereo_gray;
        std::function<void(Config*,long, stereo_t*)> callback_stereo_color;
        std::function<void(Config*,long, lidar_t*)> callback_lidar;
        std::function<void(Config*,long, gpsimu_t*)> callback_gpsimu;



    };

}


#endif //KITTI_PARSER_PARSER_H