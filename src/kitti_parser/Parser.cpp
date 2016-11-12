#include "kitti_parser/Parser.h"
#include <boost/filesystem.hpp>
#include <kitti_parser/types/stereo_t.h>
#include <kitti_parser/types/lidar_t.h>
#include <kitti_parser/types/gpsimu_t.h>


using namespace std;
using namespace kitti_parser;


/**
 * Default constructor
 * This should check to make sure that the path is valid
 * Also make sure we have the configuration information
 * Can preload all the details, so the run function can run
 */
Parser::Parser(std::string path) {

    // Save with a slash
    // http://stackoverflow.com/a/4884579
    if(*path.rbegin() == '/') {
        config.path_data = path;
    } else {
        config.path_data = path + "/";
    }

    // Check if directory
    if(!boost::filesystem::exists(config.path_data)) {
        std::cerr << "[kitti_parser]: Unable to open path" << std::endl;
        return;
    }

    // Set the configuration files
    config.path_calib_cc = config.path_data + "calib_cam_to_cam.txt";
    config.path_calib_iv = config.path_data + "calib_imu_to_velo.txt";
    config.path_calib_vc = config.path_data + "calib_velo_to_cam.txt";

    // Check to see if configuration file CAM to CAM
    if(boost::filesystem::exists(config.path_calib_cc)) {
        // TODO: Load in the config file
        // Set enabled
        config.has_calib_cc = true;
    }

    // Check to see if configuration file IMU to VELO
    if(boost::filesystem::exists(config.path_calib_iv)) {
        // TODO: Load in the config file
        // Set enabled
        config.has_calib_iv = true;
    }

    // Check to see if configuration file VELO to CAM
    if(boost::filesystem::exists(config.path_calib_vc)) {
        // TODO: Load in the config file
        // Set enabled
        config.has_calib_vc = true;
    }

    // Check to see is gray camera is there
    if(boost::filesystem::exists(config.path_data + "image_00/")
       && boost::filesystem::exists(config.path_data + "image_01/")) {
        config.has_stereo_gray = true;
    }

    // Check to see if color camera is there
    if(boost::filesystem::exists(config.path_data + "image_02/")
       && boost::filesystem::exists(config.path_data + "image_03/")) {
        config.has_stereo_color = true;
    }

    // Check to see if lidar is there
    if(boost::filesystem::exists(config.path_data + "velodyne_points/")) {
        config.has_lidar = true;
    }

    // Check to see if IMU is there
    if(boost::filesystem::exists(config.path_data + "oxts/")) {
        config.has_gpsimu = true;
    }


    // Done, load all the data
    loader = new Loader(&config);
    loader->load_all();

}


/**
 * Returns the current config file
 */
Config Parser::getConfig() {
    return config;
}


void Parser::register_callback_stereo_gray(std::function<void(Config*,long, stereo_t*)> callback) {
    callback_stereo_gray = callback;
}

void Parser::register_callback_stereo_color(std::function<void(Config *, long, stereo_t *)> callback) {
    callback_stereo_color = callback;
}

void Parser::register_callback_lidar(std::function<void(Config *, long, lidar_t *)> callback) {
    callback_lidar = callback;
}

void Parser::register_callback_gpsimu(std::function<void(Config *, long, gpsimu_t *)> callback) {
    callback_gpsimu = callback;
}

/**
 * This function will call the callback functions and pass the data
 * This can be run at double the speed, but will be limited by
 * how fast the program can process, as this is not multi-threaded
 */
void Parser::run(double time_multi) {

    // Load the first message
    Loader::message_types* next = loader->fetch_latest();

    // Loop till we run out of message to send
    // http://stackoverflow.com/a/5685578
    while(next != nullptr) {

        // Call the respective callbacks based on that type
        switch (next->which()) {
            // it's an stereo_t
            case 0: {
                stereo_t *temp_s = boost::get<stereo_t *>(*next);
                // Send, and check if valid function
                if (temp_s->is_color && callback_stereo_color){
                    callback_stereo_color.operator()(&config, temp_s->timestamp, temp_s);
                }
                // Check if function has been set
                else if(!temp_s->is_color && callback_stereo_gray) {
                    callback_stereo_gray.operator()(&config, temp_s->timestamp, temp_s);
                }

                break;
            }
            // it's a lidar_t
            case 1: {
                lidar_t *temp_v = boost::get<lidar_t *>(*next);
                // Check if function has been set
                if(callback_lidar) {
                    callback_lidar.operator()(&config, temp_v->timestamp, temp_v);
                }
                break;
            }
            // it's a gpsimu_t
            case 2: {
                gpsimu_t *temp_g = boost::get<gpsimu_t *>(*next);
                // Check if function has been set
                if(callback_gpsimu) {
                    callback_gpsimu.operator()(&config, temp_g->timestamp, temp_g);
                }
                break;
            }
        }


        // Get the next message from the loader
        next = loader->fetch_latest();
    }

}

