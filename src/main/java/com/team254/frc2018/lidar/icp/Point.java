package com.team254.frc2018.lidar.icp;

import com.team254.lib.geometry.Translation2d;

public class Point {

    public final double x, y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point(Translation2d t) {
        this(t.x(), t.y());
    }

    public Translation2d toTranslation2d() {
        return new Translation2d(x, y);
    }

    public double getDistanceSq(Point p) {
        double dx = x - p.x, dy = y - p.y;
        return dx * dx + dy * dy;
    }

    public double getDistance(Point p) {
        return Math.sqrt(getDistanceSq(p));
    }

    public String toString() {
        return "(" + x + ", " + y + ")";
    }

}