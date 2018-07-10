package com.team254.lib.trajectory;

import com.team254.lib.geometry.State;

public class TrajectoryIterator<S extends State<S>> {
    protected final TrajectoryView<S> view_;
    protected double progress_ = 0.0;
    protected TrajectorySamplePoint<S> current_sample_;

    public TrajectoryIterator(final TrajectoryView<S> view) {
        view_ = view;

        // No effect if view is empty.
        current_sample_ = view_.sample(view_.first_interpolant());
        progress_ = view_.first_interpolant();
    }

    public boolean isDone() {
        return getRemainingProgress() == 0.0;
    }

    public double getProgress() {
        return progress_;
    }

    public double getRemainingProgress() {
        return Math.max(0.0, view_.last_interpolant() - progress_);
    }

    public TrajectorySamplePoint<S> getSample() {
        return current_sample_;
    }

    public S getState() {
        return getSample().state();
    }

    public TrajectorySamplePoint<S> advance(double additional_progress) {
        progress_ = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        current_sample_ = view_.sample(progress_);
        return current_sample_;
    }

    public TrajectorySamplePoint<S> preview(double additional_progress) {
        final double progress = Math.max(view_.first_interpolant(),
                Math.min(view_.last_interpolant(), progress_ + additional_progress));
        return view_.sample(progress);
    }

    public Trajectory<S> trajectory() {
        return view_.trajectory();
    }
}
