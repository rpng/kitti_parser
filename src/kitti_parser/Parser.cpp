#include "kitti_parser/Parser.h"
#include <boost/filesystem.hpp>
#include <kitti_parser/types/stereo_t.h>


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


void Parser::register_callback_stereo_gray(std::function<void(Config*,unsigned long, stereo_t*)> callback) {
    callback_stereo_gray = callback;
}

void Parser::register_callback_stereo_color(std::function<void(Config *, unsigned long, stereo_t *)> callback) {
    callback_stereo_color = callback;
}

void Parser::register_callback_lidar(std::function<void(Config *, unsigned long, lidar_t *)> callback) {
    callback_lidar = callback;
}

void Parser::register_callback_gpsimu(std::function<void(Config *, unsigned long, gpsimu_t *)> callback) {
    callback_gpsimu = callback;
}

/**
 * This function will call the callback functions and pass the data
 * This can be run at double the speed, but will be limited by
 * how fast the program can process, as this is not multi-threaded
 */
void Parser::run(double time_multi) {


    cout << "RUN FUNCTION HAS BEEN CALLED" << endl;

    stereo_t temp;
    temp.height = 100;
    temp.width = 100;

    cout <<  loader->fetch_latest().which() << endl;
    cout <<  loader->fetch_latest().which() << endl;
    cout <<  loader->fetch_latest().which() << endl;
    cout <<  loader->fetch_latest().which() << endl;
    cout <<  loader->fetch_latest().which() << endl;

    // Call the callbacks
    callback_stereo_gray.operator()(&config,0,&temp);
    sleep(1);
    callback_stereo_gray.operator()(&config,20,&temp);
    sleep(1);
    callback_stereo_gray.operator()(&config,30,&temp);
    sleep(1);
    callback_stereo_gray.operator()(&config,40,&temp);


}

