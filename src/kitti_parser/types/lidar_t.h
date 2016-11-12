#ifndef KITTI_PARSER_LIDAR_H
#define KITTI_PARSER_LIDAR_H

namespace kitti_parser {

    typedef struct {

        unsigned long timestamp;
        unsigned long timestamp_start;
        unsigned long timestamp_end;

        int num_points;

        std::vector<float[4]> points;


    } lidar_t;

}


#endif //KITTI_PARSER_LIDAR_H