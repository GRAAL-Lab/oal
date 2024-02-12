#ifndef OAL_BOUNDING_BOX_HPP
#define OAL_BOUNDING_BOX_HPP

struct bb_data{
    // obs size
    double dim_x;
    double dim_y;
    // bb max size
    double max_x_bow;
    double max_x_stern;
    double max_y_starboard;
    double max_y_port;
    // bb safety size
    double safety_x_bow;
    double safety_x_stern;
    double safety_y_starboard;
    double safety_y_port;
    // bb uncertainty
    double gap;

    bb_data() = default;

    bb_data(double dim_x, double dim_y,
            double max_x_bow, double max_x_stern,
            double max_y_starboard, double max_y_port,
            double safety_x_bow, double safety_x_stern,
            double safety_y_starboard, double safety_y_port,
            double gap):
            dim_x(dim_x), dim_y(dim_y),
            max_x_bow(max_x_bow), max_x_stern(max_x_stern),
            max_y_starboard(max_y_starboard), max_y_port(max_y_port),
            safety_x_bow(safety_x_bow), safety_x_stern(safety_x_stern),
            safety_y_starboard(safety_y_starboard), safety_y_port(safety_y_port),
            gap(gap)
            {
              if (dim_x <= 0 || dim_y <= 0) {
                throw std::invalid_argument("Obstacle dimension must be strictly positive.");
              }

              if (max_x_bow < 1 || max_x_stern < 1 || max_y_port < 1 || max_y_starboard < 1 ||
                  safety_x_bow < 1 || safety_x_stern < 1 || safety_y_port < 1 || safety_y_starboard < 1 ||
                  gap < 0){
                throw std::invalid_argument("Invalid bounding box dimension.");
              }

              if (max_x_bow < safety_x_bow || max_x_stern < safety_x_stern || max_y_port < safety_y_port || max_y_starboard < safety_y_starboard){
                throw std::invalid_argument("Max bounding box dimension cannot be smaller than safety one.");
              }
    }

};

#endif //OAL_BOUNDING_BOX_HPP
