package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.State;
import com.team254.lib.trajectory.DistanceView;
import com.team254.lib.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;

public class TimingUtil {
    public static <S extends State<S>> Trajectory<TimedState<S>> timeParameterizeTrajectory(
            boolean reverse,
            final DistanceView<S> distance_view,
            double step_size,
            final List<TimingConstraint<S>> constraints,
            double start_velocity,
            double end_velocity,
            double max_velocity,
            double max_abs_acceleration) {
        final int num_states = (int) Math.ceil(distance_view.last_interpolant() / step_size + 1);
        List<S> states = new ArrayList<>(num_states);
        for (int i = 0; i < num_states; ++i) {
            states.add(distance_view.sample(Math.min(i * step_size, distance_view.last_interpolant())).state());
        }
        return timeParameterizeTrajectory(reverse, states, constraints, start_velocity, end_velocity,
                max_velocity, max_abs_acceleration);
    }

    public static <S extends State<S>> Trajectory<TimedState<S>> timeParameterizeTrajectory(
            boolean reverse,
            final List<S> states,
            final List<TimingConstraint<S>> constraints,
            double start_velocity,
            double end_velocity,
            double max_velocity,
            double max_abs_acceleration) {
        List<ConstrainedState<S>> constraint_states = new ArrayList<>(states.size());
        final double kEpsilon = 1e-6;

        // Forward pass. We look at pairs of consecutive states, where the start state has already been velocity
        // parameterized (though we may adjust the velocity downwards during the backwards pass). We wish to find an
        // acceleration that is admissible at both the start and end state, as well as an admissible end velocity. If
        // there is no admissible end velocity or acceleration, we set the end velocity to the state's maximum allowed
        // velocity and will repair the acceleration during the backward pass (by slowing down the predecessor).
        ConstrainedState<S> predecessor = new ConstrainedState<>();
        predecessor.state = states.get(0);
        predecessor.distance = 0.0;
        predecessor.max_velocity = start_velocity;
        predecessor.min_acceleration = -max_abs_acceleration;
        predecessor.max_acceleration = max_abs_acceleration;
        for (int i = 0; i < states.size(); ++i) {
            // Add the new state.
            constraint_states.add(new ConstrainedState<>());
            ConstrainedState<S> constraint_state = constraint_states.get(i);
            constraint_state.state = states.get(i);
            final double ds = constraint_state.state.distance(predecessor.state);
            constraint_state.distance = ds + predecessor.distance;

            // We may need to iterate to find the maximum end velocity and common acceleration, since acceleration
            // limits may be a function of velocity.
            while (true) {
                // Enforce global max velocity and max reachable velocity by global acceleration limit.
                // vf = sqrt(vi^2 + 2*a*d)
                constraint_state.max_velocity = Math.min(max_velocity,
                        Math.sqrt(predecessor.max_velocity * predecessor.max_velocity
                                + 2.0 * predecessor.max_acceleration * ds));
                if (Double.isNaN(constraint_state.max_velocity)) {
                    throw new RuntimeException();
                }
                // Enforce global max absolute acceleration.
                constraint_state.min_acceleration = -max_abs_acceleration;
                constraint_state.max_acceleration = max_abs_acceleration;

                // At this point, the state is full constructed, but no constraints have been applied aside from
                // predecessor
                // state max accel.

                // Enforce all velocity constraints.
                for (final TimingConstraint<S> constraint : constraints) {
                    constraint_state.max_velocity = Math.min(constraint_state.max_velocity,
                            constraint.getMaxVelocity(constraint_state.state));
                }
                if (constraint_state.max_velocity < 0.0) {
                    // This should never happen if constraints are well-behaved.
                    throw new RuntimeException();
                }

                // Now enforce all acceleration constraints.
                for (final TimingConstraint<S> constraint : constraints) {
                    final TimingConstraint.MinMaxAcceleration min_max_accel = constraint.getMinMaxAcceleration(
                            constraint_state.state,
                            (reverse ? -1.0 : 1.0) * constraint_state.max_velocity);
                    if (!min_max_accel.valid()) {
                        // This should never happen if constraints are well-behaved.
                        throw new RuntimeException();
                    }
                    constraint_state.min_acceleration = Math.max(constraint_state.min_acceleration,
                            reverse ? -min_max_accel.max_acceleration() : min_max_accel.min_acceleration());
                    constraint_state.max_acceleration = Math.min(constraint_state.max_acceleration,
                            reverse ? -min_max_accel.min_acceleration() : min_max_accel.max_acceleration());
                }
                if (constraint_state.min_acceleration > constraint_state.max_acceleration) {
                    // This should never happen if constraints are well-behaved.
                    throw new RuntimeException();
                }

                if (ds < kEpsilon) {
                    break;
                }
                // If the max acceleration for this constraint state is more conservative than what we had applied, we
                // need to reduce the max accel at the predecessor state and try again.
                // TODO: Simply using the new max acceleration is guaranteed to be valid, but may be too conservative.
                // Doing a search would be better.
                final double actual_acceleration = (constraint_state.max_velocity * constraint_state.max_velocity
                        - predecessor.max_velocity * predecessor.max_velocity) / (2.0 * ds);
                if (constraint_state.max_acceleration < actual_acceleration - kEpsilon) {
                    predecessor.max_acceleration = constraint_state.max_acceleration;
                } else {
                    if (actual_acceleration > predecessor.min_acceleration + kEpsilon) {
                        predecessor.max_acceleration = actual_acceleration;
                    }
                    // If actual acceleration is less than predecessor min accel, we will repair during the backward
                    // pass.
                    break;
                }
                // System.out.println("(intermediate) i: " + i + ", " + constraint_state.toString());
            }
            // System.out.println("i: " + i + ", " + constraint_state.toString());
            predecessor = constraint_state;
        }

        // Backward pass.
        ConstrainedState<S> successor = new ConstrainedState<>();
        successor.state = states.get(states.size() - 1);
        successor.distance = constraint_states.get(states.size() - 1).distance;
        successor.max_velocity = end_velocity;
        successor.min_acceleration = -max_abs_acceleration;
        successor.max_acceleration = max_abs_acceleration;
        for (int i = states.size() - 1; i >= 0; --i) {
            ConstrainedState<S> constraint_state = constraint_states.get(i);
            final double ds = constraint_state.distance - successor.distance; // will be negative.

            while (true) {
                // Enforce reverse max reachable velocity limit.
                // vf = sqrt(vi^2 + 2*a*d), where vi = successor.
                final double new_max_velocity = Math.sqrt(successor.max_velocity * successor.max_velocity
                        + 2.0 * successor.min_acceleration * ds);
                if (new_max_velocity >= constraint_state.max_velocity) {
                    // No new limits to impose.
                    break;
                }
                constraint_state.max_velocity = new_max_velocity;
                if (Double.isNaN(constraint_state.max_velocity)) {
                    throw new RuntimeException();
                }

                // Now check all acceleration constraints with the lower max velocity.
                for (final TimingConstraint<S> constraint : constraints) {
                    final TimingConstraint.MinMaxAcceleration min_max_accel = constraint.getMinMaxAcceleration(
                            constraint_state.state,
                            (reverse ? -1.0 : 1.0) * constraint_state.max_velocity);
                    if (!min_max_accel.valid()) {
                        throw new RuntimeException();
                    }
                    constraint_state.min_acceleration = Math.max(constraint_state.min_acceleration,
                            reverse ? -min_max_accel.max_acceleration() : min_max_accel.min_acceleration());
                    constraint_state.max_acceleration = Math.min(constraint_state.max_acceleration,
                            reverse ? -min_max_accel.min_acceleration() : min_max_accel.max_acceleration());
                }
                if (constraint_state.min_acceleration > constraint_state.max_acceleration) {
                    throw new RuntimeException();
                }

                if (ds > kEpsilon) {
                    break;
                }
                // If the min acceleration for this constraint state is more conservative than what we have applied, we
                // need to reduce the min accel and try again.
                // TODO: Simply using the new min acceleration is guaranteed to be valid, but may be too conservative.
                // Doing a search would be better.
                final double actual_acceleration = (constraint_state.max_velocity * constraint_state.max_velocity
                        - successor.max_velocity * successor.max_velocity) / (2.0 * ds);
                if (constraint_state.min_acceleration > actual_acceleration + kEpsilon) {
                    successor.min_acceleration = constraint_state.min_acceleration;
                } else {
                    successor.min_acceleration = actual_acceleration;
                    break;
                }
            }
            successor = constraint_state;
        }

        // Integrate the constrained states forward in time to obtain the TimedStates.
        List<TimedState<S>> timed_states = new ArrayList<>(states.size());
        double t = 0.0;
        double s = 0.0;
        double v = 0.0;
        for (int i = 0; i < states.size(); ++i) {
            final ConstrainedState<S> constrained_state = constraint_states.get(i);
            // Advance t.
            final double ds = constrained_state.distance - s;
            final double accel = (constrained_state.max_velocity * constrained_state.max_velocity - v * v) / (2.0 * ds);
            double dt = 0.0;
            if (i > 0) {
                timed_states.get(i - 1).set_acceleration(reverse ? -accel : accel);
                if (Math.abs(accel) > kEpsilon) {
                    dt = (constrained_state.max_velocity - v) / accel;
                } else if (Math.abs(v) > kEpsilon) {
                    dt = ds / v;
                } else {
                    throw new RuntimeException();
                }
            }
            t += dt;
            if (Double.isNaN(t) || Double.isInfinite(t)) {
                throw new RuntimeException();
            }

            v = constrained_state.max_velocity;
            s = constrained_state.distance;
            timed_states.add(new TimedState<>(constrained_state.state, t, reverse ? -v : v, reverse ? -accel : accel));
        }
        return new Trajectory<>(timed_states);
    }

    protected static class ConstrainedState<S extends State<S>> {
        public S state;
        public double distance;
        public double max_velocity;
        public double min_acceleration;
        public double max_acceleration;

        @Override
        public String toString() {
            return state.toString() + ", distance: " + distance + ", max_velocity: " + max_velocity + ", " +
                    "min_acceleration: " + min_acceleration + ", max_acceleration: " + max_acceleration;
        }
    }
}
