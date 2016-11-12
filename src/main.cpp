#include <iostream>
#include <kitti_parser/types/stereo_t.h>
#include "kitti_parser/Parser.h"



using namespace std;
using namespace kitti_parser;


void handle_stereo(Config* config, long timestamp, stereo_t* data);


int main(int argc, char** argv) {

    // Debug message
    cout << "[kitti_parser]: Starting up" << endl;


    // Check if there is a path to a dataset
    if(argc != 2) {
        cerr << "[kitti_parser]: Error please specify a SINGLE dataset" << endl;
        return EXIT_FAILURE;
    }

    // Parse the input
    std::string data_path = argv[1];
    std::string filename = data_path.substr(data_path.find_last_of("\\/")+1);


    // Debug message
    cout << "[kitti_parser]: Opening Dataset \"" << filename << "\""<< endl;


    // Create the parser, pass it the path
    kitti_parser::Parser parser(data_path);


    // Get the config, and display
    cout << "Current Sensor Status:" << endl;
    cout << "\tGray Stereo: " << std::boolalpha << parser.getConfig().has_stereo_gray << endl;
    cout << "\tColor Stereo: " << std::boolalpha << parser.getConfig().has_stereo_color << endl;
    cout << "\tLidar Data: " << std::boolalpha << parser.getConfig().has_lidar << endl;
    cout << "\tGPS/IMU Messages: " << std::boolalpha << parser.getConfig().has_gpsimu << endl;
    cout << "Current Config Status:" << endl;
    cout << "\tCam to Cam: " << std::boolalpha << parser.getConfig().has_calib_cc << endl;
    cout << "\tGPS/IMU to Velo: " << std::boolalpha << parser.getConfig().has_calib_iv << endl;
    cout << "\tVelo to Cam: " << std::boolalpha << parser.getConfig().has_calib_vc << endl;


    // Register the functions
    parser.register_callback_stereo_gray(&handle_stereo);
    parser.register_callback_stereo_color(&handle_stereo);

    // TODO: Start the parser at normal speed
    parser.run(1.0);


    // We are done, so return
    return EXIT_SUCCESS;

}


/**
 * Test callback function for stereo images
 */
void handle_stereo(Config* config, long timestamp, stereo_t* data) {
    cout << "Got new stereo image: " << timestamp <<
         " (" << data->width << "," << data->width << ") -> " << data->is_color << endl;
}


