#include "kitti_parser/util/Loader.h"
#include <fstream>
#include <sstream>
#include <boost/date_time.hpp>
#include <kitti_parser/types/stereo_t.h>
#include <kitti_parser/types/lidar_t.h>


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
        load_stereo(config->path_data+"image_00/",config->path_data+"image_01/",
                    time_stereo_gray, path_stereo_gray_L, path_stereo_gray_R);
    }

    // Load stereo color
    if(config->has_stereo_color) {
        load_stereo(config->path_data+"image_02/",config->path_data+"image_03/",
                    time_stereo_color, path_stereo_color_L, path_stereo_color_R);
    }


    // Load lidar timing data
    if(config->has_lidar) {
        load_lidar(config->path_data+"velodyne_points/", time_lidar_avg,
                   time_lidar_start, time_lidar_end, path_lidar);
    }


    // TODO: Do the GPS/IMU reading here
    config->has_gpsimu = false;

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

typedef boost::variant<stereo_t*, lidar_t*, gpsimu_t*> message_types;
message_types Loader::fetch_latest() {

    // Return value
    message_types next;


    // Compare stereo timestamps
    if(true) {
        next = fetch_stereo(timestamp)
    }
    // See about colored stereo timetamp
    else if(true) {

    }
    // See if LIDAR vs GPS is better
    else if(true) {

    }
    // TODO: Default to GPS/IMU measurement
    else {

    }


    return next;

}



