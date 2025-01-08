#ifndef DATASTRUCTS_HPP
#define DATASTRUCTS_HPP

#include <eigen3/Eigen/Dense>

struct Pose{
    Eigen::Vector2d position;
    double heading;
};

struct Velocity{
    //Eigen::Vector2d direction;
    double speed;
    double angle;
};

struct TPoint {
    Eigen::Vector2d position;
    double time = 0;
};

struct BoundingBoxData {
    // Set manually
    // obs size
    double dim_x;
    double dim_y;
    // bb uncertainty
    double reductionWhileCheckingPath = 5; // TODO this could depend only on the ASV capability of staying on the predicted course?

    double lookAheadSafetySpan = 1; //seconds

    // Computed 
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


    void Set( 
        double k,
        const Eigen::Vector2d& velocity, 
        std::string obs_class, 
        double size_x_sigma,
        double size_y_sigma,
        double pose_x_sigma,
        double pose_y_sigma,
        double pose_yaw_sigma,
        double vel_x_sigma,
        double vel_y_sigma){
        //assuming errors have normal distribution

        //TODO make a standard on the message covariance (cov on lat/long or x/y? cov on yaw or orientation quaternion?)

        //safety keeps in account size and velocity error
        //max also positional error
        //yaw error should reduce the difference in lenght and width

        safety_x_bow = (dim_x + k*size_x_sigma + k*vel_x_sigma*lookAheadSafetySpan)/2;
        safety_x_stern = (dim_x + k*size_x_sigma)/2;
        //std::cerr<<" dim: "<<dim_x<<"\n safety x bow: "<<safety_x_bow<<"\n safety x stern: "<<safety_x_bow<<"\n";
        safety_y_port = safety_y_starboard = (dim_y + k*size_y_sigma + k*vel_y_sigma*lookAheadSafetySpan)/2;

        max_x_bow = safety_x_bow + k*pose_x_sigma / 2;
        max_x_stern = safety_x_stern + k*pose_x_sigma / 2;
        max_y_port = max_y_starboard = safety_y_port + k*pose_y_sigma / 2;

        //reductionWhileCheckingPath = 5; 
        
        
    }

};


#endif