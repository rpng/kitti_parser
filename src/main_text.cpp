/**
 * MIT License
 *
 * Copyright (c) 2016 Patrick Geneva <pgeneva@udel.edu>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <iostream>
#include <kitti_parser/types/stereo_t.h>
#include <kitti_parser/types/lidar_t.h>
#include <kitti_parser/types/gpsimu_t.h>
#include "kitti_parser/Parser.h"



using namespace std;
using namespace kitti_parser;


void handle_stereo(Config* config, long timestamp, stereo_t* data);
void handle_lidar(Config* config, long timestamp, lidar_t* data);
void handle_gps(Config* config, long timestamp, gpsimu_t* data);

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
    parser.register_callback_lidar(&handle_lidar);
    parser.register_callback_gpsimu(&handle_gps);

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

    // Free the data once done
    delete data;
}

/**
 * Test callback function for lidar
 */
void handle_lidar(Config* config, long timestamp, lidar_t* data) {
    cout << "Got new LIDAR bin: " << timestamp << " (" << data->points.at(0).at(0) << ","
         << data->points.at(0).at(1) << "," << data->points.at(0).at(2) << "," << data->points.at(0).at(3) << ")" << endl;

    // Free the data once done
    delete data;
}


/**
 * Test callback function for GPS/IMU messages
 */
void handle_gps(Config* config, long timestamp, gpsimu_t* data) {
    cout << "Got new GPS/IMU bin: " << timestamp << " - " << data->lat << " | " << data->lon << endl;
    // Free the data once done
    delete data;
}