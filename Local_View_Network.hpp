#ifndef LOCAL_VIEW_NETWORK_HPP
#define LOCAL_VIEW_NETWORK_HPP

#include <vector>
#include "Local_View_Cell.hpp"

class Local_View_Network {
    private:
        // Constants
        const int LV_NETWORK_SIZE_MAX;
        const int LV_IMAGE_WIDTH;
        const int LV_VIEW_MATCHING_SHIFT;
        const int LV_VIEW_MATCHING_STEP;
        const double LV_VIEW_ACTIVE_DECAY;
        const double LV_VIEW_MATCHING_THRESHOLD;
        const double LV_VELOCITY_ROTATIONAL_CONSTANT;
        const double LV_VELOCITY_TRANSLATIONAL_CONSTANT;
        const double LV_VELOCITY_TRANSLATIONAL_MAX;

        // Members
        std::vector<Local_View_Cell> lv_network;
        std::vector<double> current_view;
        std::vector<double> previous_view;
        int current_view_cell_index;
        int previous_view_cell_index;
        double velocity_translational, velocity_rotational;

        double estimate_rotational_velocity(int shift);
        double estimate_translational_velocity(double average_difference);

    public:
        Local_View_Network();
        ~Local_View_Network();
        void set_current_view(const std::vector<double> &view) {
            current_view = view;
        }
        void set_previous_view(const std::vector<double> &view) {
            previous_view = view;
        }
        void set_current_view_cell_index(int index) {
            if(current_view_cell_index != index) {
                set_previous_view_cell_index(current_view_cell_index);
                lv_network[index].set_decay(LV_VIEW_ACTIVE_DECAY);
            } else {
                lv_network[index].set_decay(lv_network[index].get_decay() + LV_VIEW_ACTIVE_DECAY);
            }

            current_view_cell_index = index;
        }
        void set_previous_view_cell_index(int index) {
            if(previous_view_cell_index != index) {
                previous_view_cell_index = index;
            }
        }

        std::vector<double>& get_current_view() {
            return current_view;
        }
        std::vector<double>& get_previous_view() {
            return previous_view;
        }
        int get_current_view_cell_index() {
            return current_view_cell_index;
        }
        int get_previous_view_cell_index() {
            return previous_view_cell_index;
        }
        Local_View_Cell get_current_view_cell() {
            return lv_network[current_view_cell_index];
        }
        double get_view_matching_threshold() {
            return LV_VIEW_MATCHING_THRESHOLD;
        }
        int get_network_size() {
            return lv_network.size();
        }

        int create_cell(double pc_x, double pc_y, double pc_th);
        double match_current_view(int &best_matching_cell_index);
        bool test_match_quality(const double &average_difference);
        double compare_views(const int SHIFT, const std::vector<double> &view1, const std::vector<double> &view2);
        double estimate_velocity(int &best_matching_shift, double &rotational_velocity, double &translational_velocity);
};

#endif
