#ifndef VISUAL_ODOMETRY_MODULE_HPP
#define VISUAL_ODOMETRY_MODULE_HPP

#include <vector>

class Visual_Odometry_Module {
    private:
        static const int    VO_IMAGE_WIDTH = 320;
        static const int    VO_VIEW_MATCHING_SHIFT = VO_IMAGE_WIDTH / 4;
        static const double VO_VELOCITY_ROTATIONAL_CONSTANT = 0.0828;
        static const double VO_VELOCITY_TRANSLATIONAL_CONSTANT = 13.2;
        static const double VO_VELOCITY_TRANSLATIONAL_MAX = 18.5;

        std::vector<double> current_view;
        std::vector<double> previous_view;

    public:
        Visual_Odometry_Module();
        ~Visual_Odometry_Module();
        void set_current_view(const std::vector<double> &view);
        void set_previous_view(const std::vector<double> &view);
        std::vector<double> get_current_view();
        std::vector<double> get_previous_view();
        int estimate_movement(const std::vector<double> &view1, const std::vector<double> &view2, double &velocity_rotational, double &velocity_translational);
};

#endif
