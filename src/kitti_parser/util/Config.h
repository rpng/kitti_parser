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

#ifndef KITTI_PARSER_CONFIG_H
#define KITTI_PARSER_CONFIG_H

#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>


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


        // Store the config data here
        YAML::Node calib_cc;
        YAML::Node calib_iv;
        YAML::Node calib_vc;


    };
}

#endif //KITTI_PARSER_CONFIG_H