#define USE_CONSOLE

#include <stdio.h>
#include <allegro5/allegro.h>
#include <allegro5/allegro_primitives.h>
#include <boost/multi_array.hpp>
#include "Mapper.hpp"

Mapper::Mapper(/*dimension_xy, dimension_th*/) {}

Mapper::Mapper(matrix *pose_cell_network, const int dim_xy, const int dim_th) :
    WINDOW_WIDTH(600),
    WINDOW_HEIGHT(600) {
    network = pose_cell_network;
    dimension_xy = dim_xy;
    dimension_th = dim_th;
    cell_width = WINDOW_WIDTH / dimension_xy;
    cell_height = WINDOW_HEIGHT / dimension_xy;

    initializeDisplay();
}

Mapper::~Mapper() {
    al_destroy_display(display);
}

void Mapper::initializeDisplay() {
    ALLEGRO_DISPLAY *display = NULL;

    al_init_primitives_addon();

    if(!al_init()) {
        fprintf(stderr, "Failed to initialize Allegro!\n");
        return;
    }

    display = al_create_display(WINDOW_WIDTH, WINDOW_HEIGHT);
    al_set_window_title(display, "Pose Cell Network Activity");

    if(!display) {
        fprintf(stderr, "Failed to create display!\n");
        return;
    }
}

void Mapper::drawNetwork() {
    ALLEGRO_COLOR activity_color = al_map_rgb(0, 0, 255);
    al_clear_to_color(al_map_rgb(64,64,64));

    matrix buffer = *network;

    for(int th = 0; th < dimension_th; th++) {
        for(int x = 0; x < dimension_xy; x++) {
            for(int y = 0; y < dimension_xy; y++) {
                if(buffer[x][y][th] > 0.0) {
                    al_draw_filled_rectangle(x * cell_width, y * cell_height, x * cell_width + cell_width, y * cell_height + cell_height, activity_color);
                }
            }
        }
    }

    al_flip_display();
}
