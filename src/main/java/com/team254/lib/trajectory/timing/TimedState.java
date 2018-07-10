package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.State;
import com.team254.lib.util.Util;

import java.text.DecimalFormat;

public class TimedState<S extends State<S>> implements State<TimedState<S>> {
    protected final S state_;
    protected double t_; // Time we achieve this state.
    protected double velocity_; // ds/dt
    protected double acceleration_; // d^2s/dt^2

    public TimedState(final S state) {
        state_ = state;
    }

    public TimedState(final S state, double t, double velocity, double acceleration) {
        state_ = state;
        t_ = t;
        velocity_ = velocity;
        acceleration_ = acceleration;
    }

    public S state() {
        return state_;
    }

    public void set_t(double t) {
        t_ = t;
    }

    public double t() {
        return t_;
    }

    public void set_velocity(double velocity) {
        velocity_ = velocity;
    }

    public double velocity() {
        return velocity_;
    }

    public void set_acceleration(double acceleration) {
        acceleration_ = acceleration;
    }

    public double acceleration() {
        return acceleration_;
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return state().toString() + ", t: " + fmt.format(t()) + ", v: " + fmt.format(velocity()) + ", a: "
                + fmt.format(acceleration());
    }

    @Override
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return state().toCSV() + "," + fmt.format(t()) + "," + fmt.format(velocity()) + ","
                + fmt.format(acceleration());
    }

    @Override
    public TimedState<S> interpolate(TimedState<S> other, double x) {
        final double new_t = Util.interpolate(t(), other.t(), x);
        final double delta_t = new_t - t();
        if (delta_t < 0.0) {
            return other.interpolate(this, 1.0 - x);
        }
        boolean reversing = velocity() < 0.0 || (Util.epsilonEquals(velocity(), 0.0) && acceleration() < 0.0);
        final double new_v = velocity() + acceleration() * delta_t;
        final double new_s = (reversing ? -1.0 : 1.0) * (velocity() * delta_t + .5 * acceleration() * delta_t * delta_t);
        // System.out.println("x: " + x + " , new_t: " + new_t + ", new_s: " + new_s + " , distance: " + state()
        // .distance(other.state()));
        return new TimedState<S>(state().interpolate(other.state(), new_s / state().distance(other.state())),
                new_t,
                new_v,
                acceleration());
    }

    @Override
    public double distance(TimedState<S> other) {
        return state().distance(other.state());
    }

    @Override
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof TimedState<?>)) return false;
        TimedState<?> ts = (TimedState<?>) other;
        return state().equals(ts.state()) && Util.epsilonEquals(t(), ts.t());
    }
}
