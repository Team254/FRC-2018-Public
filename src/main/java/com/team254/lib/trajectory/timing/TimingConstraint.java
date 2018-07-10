package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.State;

public interface TimingConstraint<S extends State<S>> {
    double getMaxVelocity(S state);

    MinMaxAcceleration getMinMaxAcceleration(S state, double velocity);

    public static class MinMaxAcceleration {
        protected final double min_acceleration_;
        protected final double max_acceleration_;

        public static MinMaxAcceleration kNoLimits = new MinMaxAcceleration();

        public MinMaxAcceleration() {
            // No limits.
            min_acceleration_ = Double.NEGATIVE_INFINITY;
            max_acceleration_ = Double.POSITIVE_INFINITY;
        }

        public MinMaxAcceleration(double min_acceleration, double max_acceleration) {
            min_acceleration_ = min_acceleration;
            max_acceleration_ = max_acceleration;
        }

        public double min_acceleration() {
            return min_acceleration_;
        }

        public double max_acceleration() {
            return max_acceleration_;
        }

        public boolean valid() {
            return min_acceleration() <= max_acceleration();
        }
    }
}
