#ifndef EXPERIENCE_MAP_HPP
#define EXPERIENCE_MAP_HPP

#include <vector>
#include "Experience.hpp"

typedef struct Experience_Transition {
    int experience_from_index;
    int experience_to_index;
    double distance;
    double delta_time_s;
} Transition;

class Experience_Map {
    private:
        int current_experience_index;
        int previous_experience_index;
        double accum_delta_x, accum_delta_y, accum_delta_th;
        std::vector<Experience> experiences;
        std::vector<Transition> transitions;

    public:
        Experience_Map();
        ~Experience_Map();
        void set_current_experience_index(int experience_index) {
            this->current_experience_index = experience_index;
        }
        int get_current_experience_index() {
            return current_experience_index;
        }
        Experience* get_experience(int view_index) {
            return &this->experiences[view_index];
        }
        double get_experience_distance(int experience_index1, int experience_index2);
        int create_experience(double pc_x, double pc_y, double pc_th, double delta_time_s);
        void create_transition(int experience_from_index, int experience_to_index, double delta_time_s);
        void integrate_position(double rotational_velocity, double translational_velocity);
        void iterate();
};

#endif

