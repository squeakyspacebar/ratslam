#include <limits>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <boost/foreach.hpp>
#include "Local_View_Network.hpp"

Local_View_Network::Local_View_Network() :
    LV_NETWORK_SIZE_MAX(10000),
    LV_IMAGE_WIDTH(320),
    LV_VIEW_MATCHING_SHIFT(LV_IMAGE_WIDTH / 4),
    LV_VIEW_MATCHING_STEP(5),
    LV_VIEW_ACTIVE_DECAY(1.0),
    LV_VIEW_MATCHING_THRESHOLD(1080),
    LV_VELOCITY_ROTATIONAL_CONSTANT(0.0696848),
    LV_VELOCITY_TRANSLATIONAL_CONSTANT(0.001),
    LV_VELOCITY_TRANSLATIONAL_MAX(99999999) {
    lv_network.reserve(LV_NETWORK_SIZE_MAX);
    current_view.resize(LV_IMAGE_WIDTH);
    previous_view.resize(LV_IMAGE_WIDTH);
}

Local_View_Network::~Local_View_Network() {

}

double Local_View_Network::match_current_view(int &best_matching_cell_index) {
    if(lv_network.size() == 0) {
        return LV_VIEW_MATCHING_THRESHOLD;
    }

    double current_average_difference = 0.0;
    double minimum_average_difference = std::numeric_limits<double>::max();

    BOOST_FOREACH(Local_View_Cell view_cell, lv_network) {
        for(int shift = LV_VIEW_MATCHING_SHIFT - LV_IMAGE_WIDTH; shift <= LV_IMAGE_WIDTH - LV_VIEW_MATCHING_SHIFT; shift += LV_VIEW_MATCHING_STEP) {
            current_average_difference = compare_views(shift, current_view, view_cell.get_column_sum());

            if(current_average_difference < minimum_average_difference) {
				minimum_average_difference = current_average_difference;
				best_matching_cell_index = view_cell.get_index();
			}
        }
    }

    return minimum_average_difference;
}

bool Local_View_Network::test_match_quality(const double &average_difference) {
    if(average_difference < LV_VIEW_MATCHING_THRESHOLD) {
        return true;
    } else {
        return false;
    }
}

double Local_View_Network::compare_views(const int SHIFT, const std::vector<double> &view1, const std::vector<double> &view2) {
	double absolute_difference_sum = 0.0;
    const int ABSOLUTE_SHIFT = abs(SHIFT);

    for(int n = 0; n < LV_IMAGE_WIDTH - ABSOLUTE_SHIFT; n++) {
        absolute_difference_sum += abs(view1[n + std::max(SHIFT, 0)] - view2[n - std::min(SHIFT, 0)]);
    }

	return absolute_difference_sum / static_cast<double>(LV_IMAGE_WIDTH - ABSOLUTE_SHIFT);
}

int Local_View_Network::create_cell(double pc_x, double pc_y, double pc_th) {
	lv_network.resize(lv_network.size() + 1);
	Local_View_Cell *new_cell = &(*(lv_network.end() - 1));

	new_cell->set_index(lv_network.size() - 1);
	new_cell->set_column_sum(current_view);

	new_cell->set_pc_x(pc_x);
	new_cell->set_pc_y(pc_y);
	new_cell->set_pc_th(pc_th);
	new_cell->set_decay(LV_VIEW_ACTIVE_DECAY);

	return new_cell->get_index();
}

double Local_View_Network::estimate_velocity(int &best_matching_shift, double &rotational_velocity, double &translational_velocity) {
    best_matching_shift = 0;
    rotational_velocity = translational_velocity = 0.0;
    double current_average_difference = 0.0;
    double minimum_average_difference = std::numeric_limits<double>::max();

    for(int shift = LV_VIEW_MATCHING_SHIFT - LV_IMAGE_WIDTH; shift <= LV_IMAGE_WIDTH - LV_VIEW_MATCHING_SHIFT; shift++) {
        current_average_difference = compare_views(shift, current_view, previous_view);

        if(current_average_difference < minimum_average_difference) {
            minimum_average_difference = current_average_difference;
            best_matching_shift = shift;
        }
    }

    rotational_velocity = estimate_rotational_velocity(best_matching_shift);
    translational_velocity = estimate_translational_velocity(minimum_average_difference);

    return minimum_average_difference;
}

double Local_View_Network::estimate_rotational_velocity(int shift) {
    return shift * LV_VELOCITY_ROTATIONAL_CONSTANT;
}

double Local_View_Network::estimate_translational_velocity(double average_difference) {
    return std::min(LV_VELOCITY_TRANSLATIONAL_CONSTANT * average_difference, LV_VELOCITY_TRANSLATIONAL_MAX);
}
