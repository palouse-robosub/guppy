#include "guppy_nav/trajectory.hpp"

// public members
Trajectory::Trajectory(
    double start_velocity, double end_velocity, double attack, double decay, double total_time,
    double target_position
) :
    start_velocity(start_velocity), end_velocity(end_velocity), total_time(total_time),
    target_position(target_position) {
    attack_time = total_time * attack, decay_time = total_time * (1 - decay),
    max_velocity = compute_max_velocity();
    k1 = (max_velocity - start_velocity) / attack_time, c2 = compute_position_1(attack_time),
    c3 = compute_position_2(decay_time),
    k3 = (end_velocity - max_velocity) / (total_time - decay_time);
}

double Trajectory::get_target_velocity(double time) const {
    if (time <= attack_time)
        return (time < attack_time) ? compute_velocity_1(time) : max_velocity;
    else if (time <= decay_time)
        return max_velocity;
    else
        return (time < total_time) ? compute_velocity_3(time) : end_velocity;
}

double Trajectory::get_target_position(double time) const {
    if (time <= attack_time)
        return (time < attack_time) ? compute_position_1(time) : c2;
    else if (time <= decay_time)
        return (time < decay_time) ? compute_position_2(time) : c3;
    else
        return (time < total_time) ? compute_position_3(time) : target_position;
}

// private members
double inline Trajectory::compute_max_velocity() const {
    return (2 * target_position - attack_time * start_velocity
            - (total_time - decay_time) * end_velocity)
         / (decay_time - attack_time + total_time);
}

double inline Trajectory::compute_position_1(double time) const {
    return time * (k1 * 0.5 * time + start_velocity);
}

double inline Trajectory::compute_position_2(double time) const {
    return max_velocity * time + c2;
}

double inline Trajectory::compute_position_3(double time) const {
    const double dt = time - decay_time;

    return c3 + dt * (k3 * dt + max_velocity);
}

double inline Trajectory::compute_velocity_1(double time) const {
    return k1 * time + start_velocity;
}

double inline Trajectory::compute_velocity_3(double time) const {
    return k3 * (time - decay_time) + max_velocity;
}
