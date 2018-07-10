package com.team254.lib.trajectory;

import com.team254.lib.geometry.*;
import com.team254.lib.spline.QuinticHermiteSpline;
import com.team254.lib.spline.Spline;
import com.team254.lib.spline.SplineGenerator;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.util.Util;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtil {
    public static <S extends IPose2d<S>> Trajectory<S> mirror(final Trajectory<S> trajectory) {
        List<S> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            waypoints.add(trajectory.getState(i).mirror());
        }
        return new Trajectory<>(waypoints);
    }

    public static <S extends IPose2d<S>> Trajectory<TimedState<S>> mirrorTimed(final Trajectory<TimedState<S>> trajectory) {
        List<TimedState<S>> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            TimedState<S> timed_state = trajectory.getState(i);
            waypoints.add(new TimedState<S>(timed_state.state().mirror(), timed_state.t(), timed_state.velocity(), timed_state.acceleration()));
        }
        return new Trajectory<>(waypoints);
    }

    public static <S extends IPose2d<S>> Trajectory<S> transform(final Trajectory<S> trajectory, Pose2d transform) {
        List<S> waypoints = new ArrayList<>(trajectory.length());
        for (int i = 0; i < trajectory.length(); ++i) {
            waypoints.add(trajectory.getState(i).transformBy(transform));
        }
        return new Trajectory<>(waypoints);
    }

    /**
     * Creates a Trajectory by sampling a TrajectoryView at a regular interval.
     *
     * @param trajectory_view
     * @param interval
     * @return
     */
    public static <S extends State<S>> Trajectory<S> resample(
            final TrajectoryView<S> trajectory_view, double interval) {
        if (interval <= Util.kEpsilon) {
            return new Trajectory<S>();
        }
        final int num_states = (int) Math
                .ceil((trajectory_view.last_interpolant() - trajectory_view.first_interpolant()) / interval);
        ArrayList<S> states = new ArrayList<S>(num_states);

        for (int i = 0; i < num_states; ++i) {
            states.add(trajectory_view.sample(i * interval + trajectory_view.first_interpolant()).state());
        }
        return new Trajectory<S>(states);
    }

    public static Trajectory<Pose2dWithCurvature> trajectoryFromPathFollower(IPathFollower path_follower,
                                                                             Pose2dWithCurvature start_state, double
                                                                                     step_size, double
                                                                                     dcurvature_limit) {
        List<Pose2dWithCurvature> samples = new ArrayList<Pose2dWithCurvature>();
        samples.add(start_state);
        Pose2dWithCurvature current_state = start_state;
        while (!path_follower.isDone()) {
            // Get the desired steering command.
            final Twist2d raw_steering_command = path_follower.steer(current_state.getPose());

            // Truncate to the step size.
            Twist2d steering_command = raw_steering_command;
            if (steering_command.norm() > step_size) {
                steering_command = steering_command.scaled(step_size / steering_command.norm());
            }

            // Apply limits on spatial derivative of curvature, if desired.
            final double dcurvature = (steering_command.curvature() - current_state.getCurvature())
                    / steering_command.norm();
            final boolean curvature_valid = !Double.isNaN(dcurvature) && !Double.isInfinite(dcurvature)
                    && !Double.isNaN(current_state.getCurvature()) && !Double.isInfinite(current_state.getCurvature());
            if (dcurvature > dcurvature_limit && curvature_valid) {
                steering_command = new Twist2d(steering_command.dx, steering_command.dy,
                        (dcurvature_limit * steering_command.norm() + current_state.getCurvature())
                                * steering_command.norm());
            } else if (dcurvature < -dcurvature_limit && curvature_valid) {
                steering_command = new Twist2d(steering_command.dx, steering_command.dy,
                        (-dcurvature_limit * steering_command.norm() + current_state.getCurvature())
                                * steering_command.norm());
            }

            // Calculate the new state.
            // Use the average curvature over the interval to compute the next state.
            final Twist2d average_steering_command = !curvature_valid
                    ? steering_command
                    : new Twist2d(steering_command.dx, steering_command.dy,
                    (current_state.getCurvature() + 0.5 * dcurvature * steering_command.norm())
                            * steering_command.norm());
            current_state = new Pose2dWithCurvature(
                    current_state.getPose().transformBy(Pose2d.exp(average_steering_command)),
                    steering_command.curvature());
            if (!path_follower.isDone()) {
                samples.add(current_state);
            }
        }

        return new Trajectory<Pose2dWithCurvature>(samples);
    }

    public static Trajectory<Pose2dWithCurvature> trajectoryFromSplineWaypoints(final List<Pose2d> waypoints, double
            maxDx, double maxDy, double maxDTheta) {
        List<QuinticHermiteSpline> splines = new ArrayList<>(waypoints.size() - 1);
        for (int i = 1; i < waypoints.size(); ++i) {
            splines.add(new QuinticHermiteSpline(waypoints.get(i - 1), waypoints.get(i)));
        }
        QuinticHermiteSpline.optimizeSpline(splines);
        return trajectoryFromSplines(splines, maxDx, maxDy, maxDTheta);
    }

    public static Trajectory<Pose2dWithCurvature> trajectoryFromSplines(final List<? extends Spline> splines, double
            maxDx,
                                                                        double maxDy, double maxDTheta) {
        return new Trajectory<>(SplineGenerator.parameterizeSplines(splines, maxDx, maxDy,
                maxDTheta));
    }

    ;
}
