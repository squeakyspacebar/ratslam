#ifndef MAPPER_HPP
#define MAPPER_HPP

#include <allegro5/allegro.h>
#include <boost/multi_array.hpp>
#include <vector>

typedef boost::multi_array<double, 3> matrix;
typedef matrix::index index;

class Mapper {
    private:
        ALLEGRO_DISPLAY *display;
        matrix *network;
        int WINDOW_WIDTH;
        int WINDOW_HEIGHT;
        int dimension_xy;
        int dimension_th;
        int cell_width;
        int cell_height;

    public:
        Mapper();
        Mapper(matrix *pose_cell_network, const int dim_xy, const int dim_th);
        ~Mapper();
        void initializeDisplay();
        void drawNetwork();
};

#endif
