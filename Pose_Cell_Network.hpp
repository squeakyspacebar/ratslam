#ifndef POSE_CELL_NETWORK_HPP
#define POSE_CELL_NETWORK_HPP

#include <fstream>
#include <vector>
#include <boost/multi_array.hpp>

class Pose_Cell_Network {
    typedef boost::multi_array<double, 3> matrix;
    typedef matrix::index index;

    private:
        // Constants
        const int PC_NETWORK_DIMENSION_XY;
        const int PC_NETWORK_DIMENSION_TH;
        const int PC_CELL_SIZE_TH;
        const int PC_WEIGHT_MATRIX_XY_CONSTANT;
        const int PC_WEIGHT_MATRIX_TH_CONSTANT;
        const int PC_GLOBAL_INHIBIT_CONSTANT;

        double best_x;
        double best_y;
        double best_th;

        matrix pc_network;
        matrix excite_matrix;
        matrix inhibit_matrix;

        // Methods
        void excite();
        void inhibit();
        double cell_excite(int x, int y, int th);
        double cell_inhibit(int x, int y, int th);
        void normalize();

    public:
        Pose_Cell_Network(std::ofstream &fout);
        ~Pose_Cell_Network();
        double get_pose_cell_activity(int x, int y, int th) {
            // No bounds checking yet.
            return pc_network[x][y][th];
        }
        double get_best_x() {
            return best_x;
        }
        double get_best_y() {
            return best_y;
        }
        double get_best_th() {
            return best_th;
        }
        const int get_pc_network_dimension_xy() {
            return PC_NETWORK_DIMENSION_XY;
        }
        const int get_pc_network_dimension_th() {
            return PC_NETWORK_DIMENSION_TH;
        }
        matrix* get_network() {
            return &pc_network;
        }
        double get_delta_pc(double x, double y, double th);
        double get_min_delta(double point1, double point2, double max_distance);
        void inject(double weight, int x, int y, int th);
        void integrate_path(const double &rotational_velocity, const double &translational_velocity);
        void find_best();
        void iterate(std::ofstream &fout);
};

#endif
