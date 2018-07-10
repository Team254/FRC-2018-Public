package com.team254.lib.trajectory;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithCurvature;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.trajectory.timing.DifferentialDriveDynamicsConstraint;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingUtil;
import com.team254.lib.util.Units;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

public class IntegrationTest {

    @Test
    public void testFollowerTrajectoryGenerator() {
        // Specify desired waypoints.
        List<Translation2d> waypoints = Arrays.asList(
                new Translation2d(0.0, 0.0),
                new Translation2d(24.0, 0.0),
                new Translation2d(36.0, 0.0),
                new Translation2d(36.0, 24.0),
                new Translation2d(60.0, 24.0));

        // Create the reference trajectory (straight line motion between waypoints).
        Trajectory<Translation2d> reference_trajectory = new Trajectory<>(waypoints);

        // Generate a smooth (continuous curvature) path to follow.
        IPathFollower path_follower = new PurePursuitController<Translation2d>(
                new DistanceView<>(reference_trajectory), /* sampling_dist */1.0, /* lookahead= */ 6.0,
                /* goal_tolerance= */ 0.1);
        Trajectory<Pose2dWithCurvature> smooth_path = TrajectoryUtil.trajectoryFromPathFollower(path_follower,
                Pose2dWithCurvature.identity(), /* step_size= */ 1.0, /* dcurvature_limit= */1.0);

        assertFalse(smooth_path.isEmpty());
        System.out.println(smooth_path.toCSV());

        // Time parameterize the path subject to our dynamic constraints.
        // TODO
    }

    @Test
    public void testSplineTrajectoryGenerator() {
        // Specify desired waypoints.
        List<Pose2d> waypoints = Arrays.asList(
                new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(36.0, 0.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(60.0, 100, Rotation2d.fromDegrees(0.0)),
                new Pose2d(160.0, 100.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(200.0, 70, Rotation2d.fromDegrees(45.0)));

        // Create a trajectory from splines.
        Trajectory<Pose2dWithCurvature> trajectory = TrajectoryUtil.trajectoryFromSplineWaypoints(waypoints, 2.0,
                0.2, Math.toRadians(5.0));
        // System.out.println(trajectory.toCSV());
        // Create a differential drive.
        final double kRobotMassKg = 60.0;
        final double kRobotAngularInertia = 80.0;
        final double kRobotAngularDrag = 0.0;
        final double kWheelRadius = Units.inches_to_meters(2.0);
        DCMotorTransmission transmission = new DCMotorTransmission(
                1.0 / 0.143,
                (kWheelRadius * kWheelRadius * kRobotMassKg / 2.0) / 0.02,
                0.8);
        DifferentialDrive drive = new DifferentialDrive(kRobotMassKg, kRobotAngularInertia, kRobotAngularDrag, kWheelRadius, Units
                .inches_to_meters(26.0 / 2.0), transmission, transmission);

        // Create the constraint that the robot must be able to traverse the trajectory without ever applying more
        // than 10V.
        DifferentialDriveDynamicsConstraint<Pose2dWithCurvature> drive_constraints = new
                DifferentialDriveDynamicsConstraint<>(drive, 10.0);

        // Generate the timed trajectory.
        Trajectory<TimedState<Pose2dWithCurvature>> timed_trajectory = TimingUtil.timeParameterizeTrajectory(false, new
                        DistanceView<>(trajectory), 2.0, Arrays.asList(drive_constraints),
                0.0, 0.0, 12.0 * 14.0, 12.0 * 10.0);

        // System.out.println(timed_trajectory.toCSV());
        for (int i = 1; i < timed_trajectory.length(); ++i) {
            TrajectoryPoint<TimedState<Pose2dWithCurvature>> prev = timed_trajectory.getPoint(i - 1);
            TrajectoryPoint<TimedState<Pose2dWithCurvature>> next = timed_trajectory.getPoint(i);
            assertEquals(prev.state().acceleration(), (next.state().velocity() - prev.state().velocity()) / (next
                    .state().t() - prev.state().t()), 1E-9);
            final double dt = next.state().t() - prev.state().t();
            assertEquals(next.state().velocity(), prev.state().velocity() + prev.state().acceleration() * dt, 1E-9);
            assertEquals(next.state().distance(prev.state()), prev.state().velocity() * dt + 0.5 * prev.state()
                    .acceleration() * dt * dt, 1E-9);
        }

        // "Follow" the trajectory.
        final double kDt = 0.01;
        boolean first = true;
        TrajectoryIterator<TimedState<Pose2dWithCurvature>> it = new TrajectoryIterator<>(new TimedView<>
                (timed_trajectory));
        while (!it.isDone()) {
            TrajectorySamplePoint<TimedState<Pose2dWithCurvature>> sample;
            if (first) {
                sample = it.getSample();
                first = false;
            } else {
                sample = it.advance(kDt);
            }
            final TimedState<Pose2dWithCurvature> state = sample.state();

            final DifferentialDrive.DriveDynamics dynamics = drive.solveInverseDynamics(
                    new DifferentialDrive.ChassisState(Units.inches_to_meters(state.velocity()), state.velocity() *
                            state.state().getCurvature()),
                    new DifferentialDrive.ChassisState(Units.inches_to_meters(state.acceleration()), state
                            .acceleration() * state.state().getCurvature()));

            System.out.println(state.toCSV() + ", " + dynamics.toCSV());
        }
    }

}
