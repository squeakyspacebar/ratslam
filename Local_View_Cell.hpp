#ifndef LOCAL_VIEW_CELL_HPP
#define LOCAL_VIEW_CELL_HPP

#include <vector>
#include <algorithm>
#include <boost/multi_array.hpp>
#include "Experience.hpp"

class Local_View_Cell {
    private:
        double LVC_ASSOCIATION_CONSTANT;
        // Cell identifier.
        unsigned int index;
        // Scanline intensity profile.
        std::vector<double> column_sum;
        double decay;
        std::vector<int> experiences;
        double connection_weight;

        // Coordinates of associated pose cell.
        union {
            struct {
                double pc_x, pc_y, pc_th;
            };
            double pc[3];
        };

    public:
        Local_View_Cell();
        ~Local_View_Cell();
        void set_index(const int index) {
            this->index = index;
        }
        void set_column_sum(const std::vector<double> &column_sum) {
            this->column_sum = column_sum;
        }
        void set_decay(double decay_value) {
            decay = decay_value;
        }
        void add_experience(const int experience_index) {
            experiences.push_back(experience_index);
        }
        void set_connection_weight(const double pose_cell_activity) {
            std::max(connection_weight, LVC_ASSOCIATION_CONSTANT * 1.0 * pose_cell_activity);
        }
        void set_pc_x(const double x) {
            pc_x = x;
        }
        void set_pc_y(const double y) {
            pc_y = y;
        }
        void set_pc_th(const double th) {
            pc_th = th;
        }
        int get_index() {
            return index;
        }
        std::vector<double> get_column_sum() {
            return column_sum;
        }
        double get_decay() {
            return decay;
        }
        int get_experience_count() {
            return experiences.size();
        }
        int get_experience_index(int i) {
            return experiences[i];
        }
        double get_connection_weight() {
            return connection_weight;
        }
        double get_pc_x() {
            return pc_x;
        }
        double get_pc_y() {
            return pc_y;
        }
        double get_pc_th() {
            return pc_th;
        }
};

#endif
