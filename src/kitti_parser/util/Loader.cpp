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

#include "kitti_parser/util/Loader.h"
#include <fstream>
#include <sstream>
#include <boost/date_time.hpp>
#include <kitti_parser/types/stereo_t.h>
#include <kitti_parser/types/lidar_t.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem.hpp>
#include <kitti_parser/types/gpsimu_t.h>

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
void Loader::load_all(std::string path) {

    // Load stereo gray
    if(config->has_stereo_gray) {
        // Load it all
        load_stereo(path+"/image_00/",path+"/image_01/",
                    time_stereo_gray, path_stereo_gray_L, path_stereo_gray_R);

        // Load index values
        if(time_stereo_gray.size() > 0) {
            idx_sg = 0;
            curr_sg = time_stereo_gray.at(0);
        }

    }

    // Load stereo color
    if(config->has_stereo_color) {
        load_stereo(path+"/image_02/",path+"/image_03/",
                    time_stereo_color, path_stereo_color_L, path_stereo_color_R);

        // Load index values
        if(time_stereo_color.size() > 0) {
            idx_sc = 0;
            curr_sc = time_stereo_color.at(0);
        }
    }


    // Load lidar timing data
    if(config->has_lidar) {
        load_lidar(path+"/velodyne_points/", time_lidar_avg,
                   time_lidar_start, time_lidar_end, path_lidar);

        // Load index values
        if(time_lidar_avg.size() > 0) {
            idx_lidar = 0;
            curr_lidar = time_lidar_avg.at(0);
        }
    }


    // Do the GPS/IMU data reading here
    if(config->has_gpsimu) {
        load_gpsimu(path+"/oxts/", time_gpsimu, path_gpsimu);

        // Load index values
        if(time_gpsimu.size() > 0) {
            idx_gps = 0;
            curr_gps = time_gpsimu.at(0);
        }
    }

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
void Loader::load_timestamps(std::string path_timestamp, std::vector<long>& time, int& ct) {
    // Open the timestamp file
    std:string line;
    ifstream file_time(path_timestamp);
    // Load the timestamps
    while(getline(file_time,line)) {
        // Skip empty lines
        if(line.empty())
            continue;
        // Parse data
        // http://www.boost.org/doc/libs/1_55_0/doc/html/date_time/posix_time.html#posix_ex
        boost::posix_time::ptime pt(boost::posix_time::time_from_string(line));
        // Convert to long, subtract
        long temp = (pt - boost::posix_time::ptime{{1970,1,1}, {}}).total_milliseconds();
        // Append
        time.push_back(temp);
        // Incrememt
        ct++;
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
    int count = 0;
    load_timestamps(path_left+"timestamps.txt", time, count);

    // Loop through all sub folders, assume they are sequential
    // http://www.boost.org/doc/libs/1_47_0/libs/filesystem/v3/example/tut4.cpp
    boost::filesystem::path p1(path_left+"data/");
    vector<boost::filesystem::path> v1;
    copy(boost::filesystem::directory_iterator(p1), boost::filesystem::directory_iterator(), back_inserter(v1));
    sort(v1.begin(), v1.end());

    // Append them
    for (vector<boost::filesystem::path>::const_iterator it(v1.begin()), it_end(v1.end()); it != it_end; ++it) {
        pathL.push_back((*it).c_str());
    }

    // Loop through all sub folders, assume they are sequential
    // http://www.boost.org/doc/libs/1_47_0/libs/filesystem/v3/example/tut4.cpp
    boost::filesystem::path p2(path_right+"data/");
    vector<boost::filesystem::path> v2;
    copy(boost::filesystem::directory_iterator(p2), boost::filesystem::directory_iterator(), back_inserter(v2));
    sort(v2.begin(), v2.end());

    // Append them
    for (vector<boost::filesystem::path>::const_iterator it(v2.begin()), it_end(v2.end()); it != it_end; ++it) {
        pathR.push_back((*it).c_str());
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
    int count_a = 0;
    int count_s = 0;
    int count_e = 0;
    load_timestamps(path_lidar+"timestamps.txt", time_avg, count_a);
    load_timestamps(path_lidar+"timestamps_start.txt", time_start, count_s);
    load_timestamps(path_lidar+"timestamps_end.txt", time_end, count_e);


    // Loop through all sub folders, assume they are sequential
    // http://www.boost.org/doc/libs/1_47_0/libs/filesystem/v3/example/tut4.cpp
    boost::filesystem::path p(path_lidar+"data/");
    vector<boost::filesystem::path> v;
    copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), back_inserter(v));

    // Sort, since directory iteration
    // Is not ordered on some file systems
    sort(v.begin(), v.end());

    // Append them
    for (vector<boost::filesystem::path>::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it) {
        pathB.push_back((*it).c_str());
    }
}

/**
 * This will read in the GPS/IMU timestamp file
 * From this it will also create the paths needed
 */
void Loader::load_gpsimu(std::string path_gpsimu, std::vector<long>& time, std::vector<std::string>& path) {

    // Load the timestamps for this type
    int count = 0;
    load_timestamps(path_gpsimu+"timestamps.txt", time, count);

    // Loop through all sub folders, assume they are sequential
    // http://www.boost.org/doc/libs/1_47_0/libs/filesystem/v3/example/tut4.cpp
    boost::filesystem::path p1(path_gpsimu+"data/");
    vector<boost::filesystem::path> v1;
    copy(boost::filesystem::directory_iterator(p1), boost::filesystem::directory_iterator(), back_inserter(v1));
    sort(v1.begin(), v1.end());

    // Append them
    for (vector<boost::filesystem::path>::const_iterator it(v1.begin()), it_end(v1.end()); it != it_end; ++it) {
        path.push_back((*it).c_str());
    }
}



Loader::message_types* Loader::fetch_latest() {

    // Return value
    message_types* next;


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
        // Get next measurement
        next = new message_types(fetch_lidar(idx_lidar));
        // Move time forward
        idx_lidar++;
        curr_lidar = (idx_lidar == time_lidar_avg.size())? LONG_MAX : time_lidar_avg.at(idx_lidar);
        return next;
    }
    // See if GPS/IMU measurement is there
    else if(curr_gps != LONG_MAX) {
        // Get next measurement
        next = new message_types(fetch_gpsimu(idx_gps));
        // Move time forward
        idx_gps++;
        curr_gps = (idx_gps == time_gpsimu.size())? LONG_MAX : time_gpsimu.at(idx_gps);
        return next;
    }

    // Default, we have no measurment to give
    return nullptr;

}



/**
 * This gets both colored and gray scaled stere images
 * It loads both images and timestamp into the stereo_t data type
 */
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



/**
 * This loads velodyne point data from the LIDAR
 * This loads it into the lidar_t data type
 * Code was taken from the devkit_raw_data.zip provided by KITTI
 */
lidar_t* Loader::fetch_lidar(size_t idx) {

    // Main data type
    lidar_t* temp = new lidar_t;
    temp->timestamp = time_lidar_avg.at(idx);
    temp->timestamp_start = time_lidar_start.at(idx);
    temp->timestamp_end = time_lidar_end.at(idx);


    // allocate 4 MB buffer (only ~130*4*4 KB are needed)
    int32_t num = 1000000;
    float *data = (float*)malloc(num*sizeof(float));

    // pointers
    float *px = data+0;
    float *py = data+1;
    float *pz = data+2;
    float *pr = data+3;

    // load point cloud
    FILE *stream;
    stream = fopen(path_lidar.at(idx).c_str(),"rb");
    num = fread(data,sizeof(float),num,stream)/4;
    // Loop through and append points
    for (int32_t i=0; i<num; i++) {
        // Append
        std::array<float,4> vals = {*px,*py,*pz,*pr};
        temp->points.push_back(vals);
        // Move forward
        px+=4; py+=4; pz+=4; pr+=4;
    }
    fclose(stream);
    free(data);

    // Return
    temp->num_points = (int)temp->points.size();
    return temp;

}


gpsimu_t* Loader::fetch_gpsimu(size_t idx) {

    // Make new measurement
    gpsimu_t* next = new gpsimu_t;
    next->timestamp = time_gpsimu.at(idx);

    // Read in file of doubles
    std::vector<double> values;
    std::ifstream ifile(path_gpsimu.at(idx), std::ios::in);

    // Keep storing values from the text file so long as data exists:
    double num = 0.0;
    while (ifile >> num) {
        values.push_back(num);
    }


    // Set values, see the file for details on each one
    next->lat = values.at(0);
    next->lon = values.at(1);
    next->alt = values.at(2);
    next->roll = values.at(3);
    next->pitch = values.at(4);
    next->yaw = values.at(5);

    next->vn = values.at(6);
    next->ve = values.at(7);
    next->vf = values.at(8);
    next->vl = values.at(9);
    next->vu = values.at(10);

    next->ax = values.at(11);
    next->ay = values.at(12);
    next->az = values.at(13);
    next->af = values.at(14);
    next->al = values.at(15);
    next->au = values.at(16);
    next->wx = values.at(17);
    next->wy = values.at(18);
    next->wz = values.at(19);
    next->wf = values.at(20);
    next->wl = values.at(21);
    next->wu = values.at(22);

    next->pos_accuracy = values.at(23);
    next->vel_accuracy = values.at(24);


    next->navstat = (int)values.at(25);
    next->numsats = (int)values.at(26);
    next->posmode = (int)values.at(27);
    next->velmode = (int)values.at(28);
    next->orimode = (int)values.at(29);


    // Return it
    return next;

}

