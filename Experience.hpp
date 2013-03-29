#ifndef EXPERIENCE_HPP
#define EXPERIENCE_HPP

#include <vector>

class Experience {
    private:
        int index;
        int local_view_cell_index;

        // Coordinates.
        union {
            struct {
                double x, y, th;
            };
            double xp[3];
        };

        // Coordinates of associated pose cell.
        union {
            struct {
                double pc_x, pc_y, pc_th;
            };
            double pc[3];
        };

        std::vector<int> transitions_from;
        std::vector<int> transitions_to;

    public:
        Experience();
        ~Experience();
        void set_index(int index) {
            this->index = index;
        }
        void set_x(int x) {
            this->x = x;
        }
        void set_y(int y) {
            this->y = y;
        }
        void set_th(int th) {
            this->th = th;
        }
        void set_pc_x(int pc_x) {
            this->pc_x = pc_x;
        }
        void set_pc_y(int pc_y) {
            this->pc_y = pc_y;
        }
        void set_pc_th(int pc_th) {
            this->pc_th = pc_th;
        }
        int get_index() {
            return this->index;
        }
        int get_associated_local_view_cell_index() {
            return local_view_cell_index;
        }
        double get_pc_x() {
            return this->pc_x;
        }
        double get_pc_y() {
            return this->pc_y;
        }
        double get_pc_th() {
            return this->pc_th;
        }
        std::vector<int> get_transitions_from() {
            return transitions_from;
        }
        int get_transitions_from(int index) {
            return transitions_from[index];
        }
        std::vector<int> get_transitions_to() {
            return transitions_to;
        }
        int get_transitions_to(int index) {
            return transitions_to[index];
        }
};

#endif
