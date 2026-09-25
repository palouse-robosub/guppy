#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

class Trajectory {
private:
    double start_velocity, end_velocity, total_time, target_position;    // given
    double attack_time, decay_time, max_velocity;                        // computed
    double k1, c2, c3, k3;                                               // precomputed constants
public:
    explicit Trajectory(
        double start_velocity, double end_velocity, double attack, double decay,
        double total_time, double target_position
    );
    double get_target_velocity(double time) const;
    double get_target_position(double time) const;
  private:
    double inline compute_max_velocity() const;
    double inline compute_position_1(double time) const;
    double inline compute_position_2(double time) const;
    double inline compute_position_3(double time) const;
    double inline compute_velocity_1(double time) const;
    double inline compute_velocity_3(double time) const;
};

#endif
