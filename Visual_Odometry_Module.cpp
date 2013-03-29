#include <limits>
#include <vector>
#include <opencv2/opencv.hpp>
#include "Visual_Odometry_Module.hpp"

Visual_Odometry_Module::Visual_Odometry_Module() {
    current_view.resize(VO_IMAGE_WIDTH);
    previous_view.resize(VO_IMAGE_WIDTH);
}

Visual_Odometry_Module::~Visual_Odometry_Module() {

}

void Visual_Odometry_Module::set_current_view(const std::vector<double> &view) {
    current_view = view;
}

void Visual_Odometry_Module::set_previous_view(const std::vector<double> &view) {
    previous_view = view;
}


std::vector<double> Visual_Odometry_Module::get_current_view() {
    return current_view;
}

std::vector<double> Visual_Odometry_Module::get_previous_view() {
    return previous_view;
}

int Visual_Odometry_Module::estimate_movement(const std::vector<double> &view1, const std::vector<double> &view2,
    double &velocity_rotational, double &velocity_translational) {
    double current_difference = 0.0;
    double minimum_difference = std::numeric_limits<double>::max(); // Need to change to maximum difference.
    int best_matching_shift = 0;

    for(int shift = -VO_VIEW_MATCHING_SHIFT; shift <= VO_VIEW_MATCHING_SHIFT; shift++) {
        current_difference = 0.0;

        if(shift < 0) {
            // Negative pixel shift.
            for(int view1_column = 0, view2_column = abs(shift); view2_column < VO_IMAGE_WIDTH - 1; view1_column++, view2_column++) {
                current_difference += abs(view1[view1_column] - view2[view2_column]);
            }
        } else {
            // Positive pixel shift.
            for(int view1_column = shift, view2_column = 0; view1_column < VO_IMAGE_WIDTH - 1; view1_column++, view2_column++) {
                current_difference += abs(view1[view1_column] - view2[view2_column]);
            }
        }

        if (current_difference < minimum_difference) {
            minimum_difference = current_difference;
            best_matching_shift = abs(shift);
        } else {
            break;
        }
    }

    double avg_minimum_difference = minimum_difference / static_cast<double>(VO_IMAGE_WIDTH - VO_VIEW_MATCHING_SHIFT);
    velocity_rotational = best_matching_shift * VO_VELOCITY_ROTATIONAL_CONSTANT;
    velocity_translational = avg_minimum_difference * VO_VELOCITY_TRANSLATIONAL_CONSTANT;

    return best_matching_shift;
}
