package com.team254.lib.trajectory;

import com.team254.lib.geometry.*;
import com.team254.lib.util.Util;

public class PurePursuitController<S extends ITranslation2d<S>> implements IPathFollower {
    protected final TrajectoryIterator<S> iterator_;
    protected final double sampling_dist_;
    protected final double lookahead_;
    protected final double goal_tolerance_;
    protected boolean done_ = false;

    public PurePursuitController(final DistanceView<S> path, double sampling_dist, double lookahead,
                                 double goal_tolerance) {
        sampling_dist_ = sampling_dist;
        lookahead_ = lookahead;
        goal_tolerance_ = goal_tolerance;
        iterator_ = new TrajectoryIterator<S>(path);
    }

    public Twist2d steer(final Pose2d current_pose) {
        done_ = done_ || (iterator_.isDone()
                && current_pose.getTranslation().distance(iterator_.getState().getTranslation()) <= goal_tolerance_);
        if (isDone()) {
            return new Twist2d(0.0, 0.0, 0.0);
        }

        final double remaining_progress = iterator_.getRemainingProgress();
        double goal_progress = 0.0;
        // Find the first point > lookahead distance away from current_pose, or the last point otherwise.
        for (double progress = 0.0; progress <= remaining_progress; progress = Math.min(remaining_progress,
                progress + sampling_dist_)) {
            double dist = current_pose.getTranslation().distance(iterator_.preview(progress).state().getTranslation());
            if (dist > lookahead_) {
                if (goal_progress == 0.0 && !iterator_.isDone()) {
                    // Make sure we don't get stuck due to numerical issues when sampling dist is large relative to
                    // lookahead.
                    goal_progress = progress;
                }
                break;
            }
            goal_progress = progress;
            if (progress == remaining_progress) {
                break;
            }
        }
        iterator_.advance(goal_progress);
        final Arc<S> arc = new Arc<S>(current_pose, iterator_.getState());
        if (arc.length < Util.kEpsilon) {
            return new Twist2d(0.0, 0.0, 0.0);
        } else {
            return new Twist2d(arc.length, 0.0, arc.length / arc.radius);
        }
    }

    public boolean isDone() {
        return done_;
    }

    protected static <S extends ITranslation2d<S>> double getDirection(Pose2d pose, S point) {
        Translation2d poseToPoint = new Translation2d(pose.getTranslation(), point.getTranslation());
        Translation2d robot = pose.getRotation().toTranslation();
        double cross = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x();
        return (cross < 0.) ? -1. : 1.; // if robot < pose turn left
    }

    public static class Arc<S extends ITranslation2d<S>> {
        public Translation2d center;
        public double radius;
        public double length;

        public Arc(final Pose2d pose, final S point) {
            center = findCenter(pose, point);
            radius = new Translation2d(center, point.getTranslation()).norm();
            length = findLength(pose, point, center, radius);
            radius *= getDirection(pose, point);
        }

        protected Translation2d findCenter(Pose2d pose, S point) {
            final Translation2d poseToPointHalfway = pose.getTranslation().interpolate(point.getTranslation(), 0.5);
            final Rotation2d normal = pose.getTranslation().inverse().translateBy(poseToPointHalfway).direction()
                    .normal();
            final Pose2d perpendicularBisector = new Pose2d(poseToPointHalfway, normal);
            final Pose2d normalFromPose = new Pose2d(pose.getTranslation(),
                    pose.getRotation().normal());
            if (normalFromPose.isColinear(perpendicularBisector.normal())) {
                // Special case: center is poseToPointHalfway.
                return poseToPointHalfway;
            }
            return normalFromPose.intersection(perpendicularBisector);
        }

        protected double findLength(Pose2d pose, S point, Translation2d center, double radius) {
            if (radius < Double.MAX_VALUE) {
                final Translation2d centerToPoint = new Translation2d(center, point.getTranslation());
                final Translation2d centerToPose = new Translation2d(center, pose.getTranslation());
                // If the point is behind pose, we want the opposite of this angle. To determine if the point is behind,
                // check the sign of the cross-product between the normal vector and the vector from pose to point.
                final boolean behind = Math.signum(
                        Translation2d.cross(pose.getRotation().normal().toTranslation(),
                                new Translation2d(pose.getTranslation(), point.getTranslation()))) > 0.0;
                final Rotation2d angle = Translation2d.getAngle(centerToPose, centerToPoint);
                return radius * (behind ? 2.0 * Math.PI - Math.abs(angle.getRadians()) : Math.abs(angle.getRadians()));
            } else {
                return new Translation2d(pose.getTranslation(), point.getTranslation()).norm();
            }
        }
    }
}
