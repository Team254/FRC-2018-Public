package com.team254.frc2018.lidar;

import com.team254.frc2018.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Represents a single point from the LIDAR sensor. This consists of
 * an angle, distance, and timestamp.
 */
class LidarPoint {
    public final double timestamp;
    public final double angle;
    public final double distance;
    private RobotState mRobotState = RobotState.getInstance();

    private final static int MAX_ENTRIES = 10;
    private final static LinkedHashMap<Double, Pose2d> mRobotPoseMap = new LinkedHashMap<Double, Pose2d>() {
        @Override
        protected boolean removeEldestEntry(Map.Entry<Double, Pose2d> eldest) {
            return this.size() > MAX_ENTRIES;
        }
    };

    public static final double MM_TO_IN = 1 / 25.4; // 1 inch = 25.4 millimeters

    public LidarPoint(double timestamp, double angle, double distance) {
        this.timestamp = timestamp;
        this.angle = angle;
        this.distance = distance * MM_TO_IN;
    }

    /**
     * Convert this point into a {@link Translation2d} in cartesian (x, y)
     * coordinates. The point's timestamp is used along with the {@link RobotState}
     * to take into account the robot's pose at the time the point was detected.
     */
    public Translation2d toCartesian() {
        // convert the polar coords to cartesian coords
        double radians = Math.toRadians(angle);
        Translation2d cartesian = new Translation2d(Math.cos(radians) * distance, Math.sin(radians) * distance);

        // transform by the robot's pose
        Pose2d robotPose;
        if (mRobotPoseMap.containsKey(timestamp)) {
            robotPose = mRobotPoseMap.get(timestamp);
        } else {
            robotPose = mRobotState.getFieldToLidar(timestamp);
            mRobotPoseMap.put(timestamp, robotPose);
        }

        robotPose.transformBy(Pose2d.fromTranslation(cartesian));
        return robotPose.getTranslation();
    }
}