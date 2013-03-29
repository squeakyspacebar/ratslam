#ifndef RATSLAM_HPP
#define RATSLAM_HPP

class RatSlam {
    private:
        double rotational_velocity;
        double translational_velocity;
        double time_delta;
        double time_s;

    public:
        RatSlam();
        ~RatSlam();
        void set_odometric_data(double rotational_velocity, double translational_velocity) {
            this->rotational_velocity = rotational_velocity;
            this->translational_velocity = translational_velocity;
        }
        void update_time(double delta_time_s) {
            this->time_delta = delta_time_s;
            this->time_s += delta_time_s;
        }
};

#endif
