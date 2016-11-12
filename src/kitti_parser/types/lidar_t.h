#ifndef KITTI_PARSER_LIDAR_H
#define KITTI_PARSER_LIDAR_H

namespace kitti_parser {

    typedef struct {

        long timestamp;
        long timestamp_start;
        long timestamp_end;

        int num_points;

        std::vector<float[4]> points;


    } lidar_t;

}


#endif //KITTI_PARSER_LIDAR_H