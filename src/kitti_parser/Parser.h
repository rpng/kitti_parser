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