#include <cmath>
#include <fstream>
#include "Pose_Cell_Network.hpp"

Pose_Cell_Network::Pose_Cell_Network(std::ofstream &fout) :
    PC_NETWORK_DIMENSION_XY(30),
    PC_NETWORK_DIMENSION_TH(36),
    PC_CELL_SIZE_TH(2.0 * M_PI / PC_NETWORK_DIMENSION_TH), // Cell rotational size given in radians.
    PC_WEIGHT_MATRIX_XY_CONSTANT(7),
    PC_WEIGHT_MATRIX_TH_CONSTANT(7),
    PC_GLOBAL_INHIBIT_CONSTANT(0.00002),
    pc_network(boost::extents[PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_TH]),
    excite_matrix(boost::extents[PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_TH]),
    inhibit_matrix(boost::extents[PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_TH]) {
    // Initializes the pose cell activity levels and constructs lookup matrices for excitatory/inhibitory weights.
	for(int c = 0; c < PC_NETWORK_DIMENSION_TH; c++) {
		for(int b = 0; b < PC_NETWORK_DIMENSION_XY; b++) {
			for(int a = 0; a < PC_NETWORK_DIMENSION_XY; a++) {
			    pc_network[a][b][c] = 0.0;
				excite_matrix[a][b][c] = exp(-(a * a + b * b) / PC_WEIGHT_MATRIX_XY_CONSTANT) *
                    exp(-(c * c) / PC_WEIGHT_MATRIX_TH_CONSTANT);
                double inhibit = exp(-(a * a + b * b) / PC_WEIGHT_MATRIX_XY_CONSTANT) *
                    exp(-(c * c) / PC_WEIGHT_MATRIX_TH_CONSTANT);
                if(inhibit == 1.0) {
                    inhibit_matrix[a][b][c] = 0.0;
                } else {
                    inhibit_matrix[a][b][c] = -1.0 / inhibit;
                }
                fout << "Excite "  << a << " " << b << " " << c << ": " << excite_matrix[a][b][c] << std::endl;
                fout << "Inhibit "  << a << " " << b << " " << c << ": " << inhibit_matrix[a][b][c] << std::endl;
			}
		}
	}

    // the starting position within the posecell network
	best_x = floor((double)PC_NETWORK_DIMENSION_XY/2.0);
	best_y = floor((double)PC_NETWORK_DIMENSION_XY/2.0);
	best_th = floor((double)PC_NETWORK_DIMENSION_TH/2.0);

	pc_network[best_x][best_y][best_th] = 1.0;
}

Pose_Cell_Network::~Pose_Cell_Network() {

}

void Pose_Cell_Network::integrate_path(const double &rotational_velocity, const double &translational_velocity) {
    if(abs(rotational_velocity) > 36 || abs(translational_velocity) > 30) {
        return;
    }

    double shift_th = rotational_velocity;
    double shift_x = translational_velocity * std::cos(rotational_velocity);
    double shift_y = translational_velocity * std::sin(rotational_velocity);

    int new_th = (static_cast<int>(best_th + shift_th) + PC_NETWORK_DIMENSION_TH) % PC_NETWORK_DIMENSION_TH;
    int new_x = (static_cast<int>(best_x + shift_x) + PC_NETWORK_DIMENSION_XY) % PC_NETWORK_DIMENSION_XY;
    int new_y = (static_cast<int>(best_y + shift_y) + PC_NETWORK_DIMENSION_XY) % PC_NETWORK_DIMENSION_XY;

    pc_network[new_x][new_y][new_th] = pc_network[best_x][best_y][best_th];
}

void Pose_Cell_Network::find_best() {
    double maximum_activity = 0.0;
    double x = 0.0, y = 0.0, th = 0.0;

    for(int k = 0; k < PC_NETWORK_DIMENSION_TH; k++) {
        for(int j = 0; j < PC_NETWORK_DIMENSION_XY; j++) {
            for(int i = 0; i < PC_NETWORK_DIMENSION_XY; i++) {
                if(pc_network[i][j][k] > maximum_activity) {
                    maximum_activity = pc_network[i][j][k];
                    x = static_cast<double>(i);
                    y = static_cast<double>(j);
                    th = static_cast<double>(k);
                }
            }
        }
    }

    best_x = x;
    best_y = y;
    best_th = th;
}

void Pose_Cell_Network::inject(double weight, int x, int y, int th) {
    if((x >= 0) && (y >= 0) && (th >= 0) && !(x > PC_NETWORK_DIMENSION_XY - 1) && !(y > PC_NETWORK_DIMENSION_XY - 1) && !(th > PC_NETWORK_DIMENSION_TH - 1)) {
        pc_network[x][y][th] += weight;
    }
}

// Scans through the network for cells to excite.
void Pose_Cell_Network::excite() {
    matrix excite_delta_matrix(boost::extents[PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_TH]);

    // Calculate the change in activity due to excitation for each cell.
    for(int k = 0; k < PC_NETWORK_DIMENSION_TH; k++) {
        for(int j = 0; j < PC_NETWORK_DIMENSION_XY; j++) {
            for(int i = 0; i < PC_NETWORK_DIMENSION_XY; i++) {
                if(pc_network[i][j][k] != 0.0) {
                    excite_delta_matrix[i][j][k] = cell_excite(i, j, k);
                }
            }
        }
    }

    // Apply the excitation activity to each cell.
    for(int k = 0; k < PC_NETWORK_DIMENSION_TH; k++) {
        for(int j = 0; j < PC_NETWORK_DIMENSION_XY; j++) {
            for(int i = 0; i < PC_NETWORK_DIMENSION_XY; i++) {
                pc_network[i][j][k] += excite_delta_matrix[i][j][k];
            }
        }
    }
}

// Scans through the network for cells to inhibit.
// Combines local and global inhibition.
void Pose_Cell_Network::inhibit() {
    matrix inhibit_delta_matrix(boost::extents[PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_XY][PC_NETWORK_DIMENSION_TH]);

    // Calculate the change in activity due to inhibition for each cell.
    for(int k = 0; k < PC_NETWORK_DIMENSION_TH; k++) {
        for(int j = 0; j < PC_NETWORK_DIMENSION_XY; j++) {
            for(int i = 0; i < PC_NETWORK_DIMENSION_XY; i++) {
                if(pc_network[i][j][k] != 0.0) {
                    inhibit_delta_matrix[i][j][k] = cell_inhibit(i, j, k);
                }
            }
        }
    }

    // Apply the excitation activity to each cell.
    for(int k = 0; k < PC_NETWORK_DIMENSION_TH; k++) {
        for(int j = 0; j < PC_NETWORK_DIMENSION_XY; j++) {
            for(int i = 0; i < PC_NETWORK_DIMENSION_XY; i++) {
                pc_network[i][j][k] += inhibit_delta_matrix[i][j][k];
            }
        }
    }
}

// Calculates excitement for a particular cell.
double Pose_Cell_Network::cell_excite(int x, int y, int th) {
    double activity_delta = 0.0;

    for(int k = 0; k < PC_NETWORK_DIMENSION_TH - 1; k++) {
        int c = abs((th - k) % PC_NETWORK_DIMENSION_TH);

        for(int j = 0; j < PC_NETWORK_DIMENSION_XY - 1; j++) {
            int b = abs((y - j) % PC_NETWORK_DIMENSION_XY);

            for(int i = 0; i < PC_NETWORK_DIMENSION_XY - 1; i++) {
                int a = abs((x - i) % PC_NETWORK_DIMENSION_XY);

                activity_delta += pc_network[i][j][k] * excite_matrix[a][b][c];
            }
        }
    }

    return activity_delta;
}

// Calculates inhibition for a particular cell.
double Pose_Cell_Network::cell_inhibit(int x, int y, int th) {
    double activity_delta = 0.0;

    for(int k = 0; k < PC_NETWORK_DIMENSION_TH; k++) {
        int c = abs((th - k) % PC_NETWORK_DIMENSION_TH);

        for(int j = 0; j < PC_NETWORK_DIMENSION_XY; j++) {
            int b = abs((y - j) % PC_NETWORK_DIMENSION_XY);

            for(int i = 0; i < PC_NETWORK_DIMENSION_XY; i++) {
                int a = abs((x - i) % PC_NETWORK_DIMENSION_XY);

                activity_delta += pc_network[i][j][k] * inhibit_matrix[a][b][c];
            }
        }
    }

    return activity_delta;
}

void Pose_Cell_Network::normalize() {
    // Technically - the way it's represented here - not quite a normalizing constant.
    double normalizing_constant = 0.0;

    // Eliminate negative activity levels and construct the normalization constant.
    for(int k = 0; k < PC_NETWORK_DIMENSION_TH; k++) {
        for(int j = 0; j < PC_NETWORK_DIMENSION_XY; j++) {
            for(int i = 0; i < PC_NETWORK_DIMENSION_XY; i++) {
                if(pc_network[i][j][k] < 0.0) {
                    pc_network[i][j][k] = 0.0;
                } else {
                    normalizing_constant += pc_network[i][j][k];
                }
            }
        }
    }

    // Check whether or not the normalizing constant is mathematically defined.
    // Cheap error handling - could [should] be improved.
    if(!(normalizing_constant > 0.0)) {
        return;
    }

    // Normalize all activity levels in the network.
    for(int k = 0; k < PC_NETWORK_DIMENSION_TH; k++) {
        for(int j = 0; j < PC_NETWORK_DIMENSION_XY; j++) {
            for(int i = 0; i < PC_NETWORK_DIMENSION_XY; i++) {
                pc_network[i][j][k] /= normalizing_constant;
            }
        }
    }
}

double Pose_Cell_Network::get_delta_pc(double x, double y, double th) {
    return sqrt(pow(get_min_delta(best_x, x, PC_NETWORK_DIMENSION_XY), 2) +
        pow(get_min_delta(best_y, y, PC_NETWORK_DIMENSION_XY), 2) +
        pow(get_min_delta(best_th, th, PC_NETWORK_DIMENSION_TH), 2));
}

double Pose_Cell_Network::get_min_delta(double point1, double point2, double max_distance) {
    double absolute_distance = abs(point1 - point2);
    return std::min(absolute_distance, max_distance - absolute_distance);
}

void Pose_Cell_Network::iterate(std::ofstream &fout) {
    excite();
    inhibit();
    //normalize();
}
