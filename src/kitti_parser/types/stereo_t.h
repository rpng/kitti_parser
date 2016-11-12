#ifndef KITTI_PARSER_STEREO_H
#define KITTI_PARSER_STEREO_H

#include <opencv2/core/mat.hpp>

namespace kitti_parser {

    typedef struct {

        long timestamp;

        bool is_color;

        int width;
        int height;

        cv::Mat image_left;
        cv::Mat image_right;

    } stereo_t;

}


#endif //KITTI_PARSER_STEREO_H