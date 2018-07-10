package com.team254.frc2018.lidar;

import java.util.LinkedHashMap;
import java.util.Map;

import com.team254.frc2018.RobotState;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;

/**
 * Represents a single point from the lidar
 */
class LidarPoint {
    double timestamp;
    double angle;
    double distance;
    private double excludeRectMinX, excludeRectMinY, excludeRectMaxX, excludeRectMaxY; // TODO set values
    private RobotState mRobotState = RobotState.getInstance();
    private final static int MAX_ENTRIES = 10;
    private final static LinkedHashMap<Double, Pose2d> mRobotPoseMap = new LinkedHashMap<Double, Pose2d>() {
        @Override
        protected boolean removeEldestEntry(Map.Entry<Double, Pose2d> eldest) {
            return this.size() > MAX_ENTRIES;
        }
    };

    public LidarPoint(double timestamp, double angle, double distance) {
        this.timestamp = timestamp;
        this.angle = angle;
        this.distance = distance;
    }

    public Translation2d toCartesian() {
        double radians = Math.toRadians(angle);
        Translation2d cartesian = new Translation2d(Math.cos(radians) * distance, Math.sin(radians) * distance);

        if (cartesian.x() < excludeRectMinX || cartesian.y() < excludeRectMinY || cartesian.x() > excludeRectMaxX ||
                cartesian.y() > excludeRectMaxY) {
            return null;
        }

        Pose2d robotPose;
        if (mRobotPoseMap.containsKey(timestamp)) {
            robotPose = mRobotPoseMap.get(timestamp);
        } else {
            robotPose = mRobotState.getFieldToVehicle(timestamp);
            mRobotPoseMap.put(timestamp, robotPose);
        }

        robotPose.transformBy(Pose2d.fromTranslation(cartesian));
        return robotPose.getTranslation();
    }
}