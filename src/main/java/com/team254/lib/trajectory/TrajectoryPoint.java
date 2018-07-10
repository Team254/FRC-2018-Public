package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;

public class TrajectoryPoint<S extends State<S>> {
    protected final S state_;
    protected final int index_;

    public TrajectoryPoint(final S state, int index) {
        state_ = state;
        index_ = index;
    }

    public S state() {
        return state_;
    }

    public int index() {
        return index_;
    }
}
