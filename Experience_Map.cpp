#include <cmath>
#include "Experience_Map.hpp"

Experience_Map::Experience_Map() {
    experiences.reserve(10000);
    transitions.reserve(10000);

	current_experience_index = 0;
	previous_experience_index = 0;

	accum_delta_th = M_PI/2.0;
	accum_delta_x = 0;
	accum_delta_y = 0;
}

Experience_Map::~Experience_Map() {
	transitions.clear();
	experiences.clear();
}

double Experience_Map::get_experience_distance(int experience_index1, int experience_index2) {
    return 0.0;
}

int Experience_Map::create_experience(double pc_x, double pc_y, double pc_th, double delta_time_s) {
    experiences.resize(experiences.size() + 1);
    Experience *new_experience = &(*(experiences.end() - 1));

    new_experience->set_index(experiences.size() - 1);
	if(new_experience->get_index() == 0) {
		new_experience->set_x(0);
		new_experience->set_y(0);
		new_experience->set_th(0);
	} else {
		new_experience->set_x(experiences[current_experience_index].x + this->accum_delta_x);
		new_experience->set_y(experiences[current_experience_index].y + this->accum_delta_y);
		new_experience->set_th(experiences[current_experience_index].th + this->accum_delta_th); //gri::clip_rad_180(accum_delta_facing);
	}

	new_experience->set_pc_x(pc_x);
	new_experience->set_pc_y(pc_y);
	new_experience->set_pc_th(pc_th);

	return new_experience->get_index();
}

void Experience_Map::create_transition(int experience_from_index, int experience_to_index, double delta_time_s) {
	Experience *current_exp = &experiences[experience_from_index];

	// check if the link already exists
	for(unsigned int i = 0; i < experiences[experience_from_index].get_transitions_from().size(); i++) {
		if(transitions[experiences[current_experience_index].get_transitions_from(i)].experience_to_index == experience_to_index) {
			return;
		}
	}

	transitions.resize(transitions.size() + 1);
	Transition *new_transition = &(*(transitions.end() - 1));

	new_transition->experience_to_index = experience_to_index;
	new_transition->experience_from_index = experience_from_index;
	new_transition->distance = sqrt(accum_delta_x * accum_delta_x + accum_delta_y * accum_delta_y);
	//new_link->heading_rad = gri::get_signed_delta_rad(current_exp->th_rad, atan2(accum_delta_y, accum_delta_x));
	//new_link->facing_rad = gri::get_signed_delta_rad(current_exp->th_rad, accum_delta_facing);
	new_transition->delta_time_s = delta_time_s;

	// add this link to the 'to exp' so we can go backwards through the em
	experiences[experience_from_index].get_transitions_from().push_back(transitions.size() - 1);
	experiences[experience_to_index].get_transitions_to().push_back(transitions.size() - 1);

	return;
}

void Experience_Map::integrate_position(double rotational_velocity, double translational_velocity) {
	//accum_delta_facing = accum_delta_facing + rotational_velocity;
	accum_delta_x = accum_delta_x + translational_velocity * cos(rotational_velocity);
	accum_delta_y = accum_delta_y + translational_velocity * sin(rotational_velocity);
}

void Experience_Map::iterate() {

}
