#include "kitti_parser/util/Loader.h"
#include <fstream>
#include <sstream>
#include <boost/date_time.hpp>
#include <kitti_parser/types/stereo_t.h>
#include <kitti_parser/types/lidar_t.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace kitti_parser;

/**
 * Default constructor for the loader
 * Just saves the config file information
 */
Loader::Loader(Config* conf) {
    config = conf;
}


/**
 * This will load all the timestamps and paths
 * If the sensor data is not there, then we will skip
 */
void Loader::load_all() {

    // Load stereo gray
    if(config->has_stereo_gray) {
        // Load it all
        load_stereo(config->path_data+"image_00/",config->path_data+"image_01/",
                    time_stereo_gray, path_stereo_gray_L, path_stereo_gray_R);

        // Load index values
        if(time_stereo_gray.size() > 0) {
            idx_sg = 0;
            curr_sg = time_stereo_gray.at(0);
        }

    }

    // Load stereo color
    if(config->has_stereo_color) {
        load_stereo(config->path_data+"image_02/",config->path_data+"image_03/",
                    time_stereo_color, path_stereo_color_L, path_stereo_color_R);

        // Load index values
        if(time_stereo_color.size() > 0) {
            idx_sc = 0;
            curr_sc = time_stereo_color.at(0);
        }
    }


    // Load lidar timing data
    if(config->has_lidar) {
        load_lidar(config->path_data+"velodyne_points/", time_lidar_avg,
                   time_lidar_start, time_lidar_end, path_lidar);

        // Load index values
        if(time_lidar_avg.size() > 0) {
            idx_lidar = 0;
            curr_lidar = time_lidar_avg.at(0);
        }
    }


    // TODO: Do the GPS/IMU reading here
    config->has_gpsimu = false;
    config->has_lidar = false;

    // Debug
    //cout << "Loaded the following files:" << endl;
    //cout << "\tStereo Gray: " << time_stereo_gray.size() << endl;
    //cout << "\tStereo Color: " << time_stereo_color.size() << endl;
    //cout << "\tLidar Data: " << time_lidar_avg.size() << endl;
    //cout << "\tGPS/IMU Data: " << 0 << endl;

}


/**
 * Loads a timestamp file based on the path given
 */
void Loader::load_timestamps(std::string path_timestamp, std::vector<long>& time) {
    // Open the timestamp file
    std:string line;
    ifstream file_time(path_timestamp);
    // Load the timestamps
    while(getline(file_time,line)) {
        // Parse data
        // http://www.boost.org/doc/libs/1_55_0/doc/html/date_time/posix_time.html#posix_ex
        boost::posix_time::ptime pt(boost::posix_time::time_from_string(line));
        // Convert to long, subtract
        long temp = (pt - boost::posix_time::ptime{{1970,1,1}, {}}).total_milliseconds();
        // Append
        time.push_back(temp);
        // Debug
        //cout << line << " => " << temp << endl;
    }
    // Close file
    file_time.close();
}


/**
 * This will read in the timestamp file
 * From this it will also create the paths needed
 */
void Loader::load_stereo(std::string path_left, std::string path_right, std::vector<long>& time,
                         std::vector<std::string>& pathL, std::vector<std::string>& pathR) {

    // Load the timestamps for this type
    load_timestamps(path_left+"timestamps.txt", time);

    // Create the paths
    for(int i=0; i<time.size(); i++) {
        // Create filename => (0000000000.png)
        // http://stackoverflow.com/a/6143897
        std::ostringstream ss;
        ss << std::setw(10) << std::setfill('0') << i;
        // Append paths
        pathL.push_back(path_left+"data/"+ss.str()+".png");
        pathR.push_back(path_right+"data/"+ss.str()+".png");
        // Debug
        //cout << path_left+"data/"+ss.str()+".png" << endl;
    }

}


/**
 * This will read in the LIDAR timestamp file
 * From this it will also create the paths needed
 */
void Loader::load_lidar(std::string path_lidar,
                        std::vector<long>& time_avg, std::vector<long>& time_start,
                        std::vector<long>& time_end, std::vector<std::string>& pathB) {

    // Load the timestamps for this type
    load_timestamps(path_lidar+"timestamps.txt", time_avg);
    load_timestamps(path_lidar+"timestamps_start.txt", time_start);
    load_timestamps(path_lidar+"timestamps_end.txt", time_end);

    // Create the paths
    for(int i=0; i<time_avg.size(); i++) {
        // Create filename => (0000000000.bin)
        // http://stackoverflow.com/a/6143897
        std::ostringstream ss;
        ss << std::setw(10) << std::setfill('0') << i;
        // Append paths
        pathB.push_back(path_lidar+"data/"+ss.str()+".bin");
        // Debug
        //cout << path_lidar+"data/"+ss.str()+".bin" << endl;
    }

}

Loader::message_types* Loader::fetch_latest() {

    // Return value
    message_types* next ;


    // Compare stereo timestamps
    if((curr_sg < curr_sc || !config->has_stereo_color)
       && (curr_sg < curr_lidar || !config->has_lidar)
       && (curr_sg < curr_gps || !config->has_gpsimu) && curr_sg != LONG_MAX) {
        // Get the next measurement
        next = new message_types(fetch_stereo(idx_sg, false));
        // Move time forward
        idx_sg++;
        curr_sg = (idx_sg == time_stereo_gray.size())? LONG_MAX : time_stereo_gray.at(idx_sg);
        return next;
    }
    // See about colored stereo timetamp
    else if((curr_sc < curr_lidar || !config->has_lidar)
            && (curr_sc < curr_gps || !config->has_gpsimu) && curr_sc != LONG_MAX) {
        // Get the next measurement
        next = new message_types(fetch_stereo(idx_sc, true));
        // Move time forward
        idx_sc++;
        curr_sc = (idx_sc == time_stereo_color.size())? LONG_MAX : time_stereo_color.at(idx_sc);
        return next;
    }
    // See if LIDAR vs GPS is better
    else if((curr_lidar < curr_gps || !config->has_gpsimu) && curr_lidar != LONG_MAX) {

        // Move time forward
        idx_lidar++;
        curr_lidar = (idx_lidar == time_lidar_avg.size())? LONG_MAX : time_lidar_avg.at(idx_lidar);
        //return next;
    }
    // See if GPS/IMU measurement is there
    else if(curr_gps != LONG_MAX) {
        // Move time forward
        //idx_gps++;
        //curr_gps = (idx_gps == time.size())? LONG_MAX : time.at(idx_gps);
        //return next;
    }

    // Default, we have no measurment to give
    return nullptr;

}




stereo_t* Loader::fetch_stereo(size_t idx, bool is_color) {

    stereo_t* next = new stereo_t;

    if(is_color) {
        next->is_color = true;
        next->timestamp = time_stereo_color.at(idx);
        next->image_left = cv::imread(path_stereo_color_L.at(idx), CV_LOAD_IMAGE_COLOR);
        next->image_right = cv::imread(path_stereo_color_R.at(idx), CV_LOAD_IMAGE_COLOR);
        next->width = next->image_left.cols;
        next->height = next->image_left.rows;
    } else {
        next->is_color = false;
        next->timestamp = time_stereo_gray.at(idx);
        next->image_left = cv::imread(path_stereo_gray_L.at(idx), CV_LOAD_IMAGE_GRAYSCALE);
        next->image_right = cv::imread(path_stereo_gray_R.at(idx), CV_LOAD_IMAGE_GRAYSCALE);
        next->width = next->image_left.cols;
        next->height = next->image_left.rows;
    }


    // Return
    return next;

}