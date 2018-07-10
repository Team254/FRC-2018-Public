package com.team254.lib.spline;

import com.team254.lib.geometry.*;

import java.util.ArrayList;
import java.util.List;

public class SplineGenerator {
    private static final double kMaxDX = 2.0; //inches
    private static final double kMaxDY = 0.05; //inches
    private static final double kMaxDTheta = 0.1; //radians!
    private static final int kMinSampleSize = 1;

    /**
     * Converts a spline into a list of Twist2d's.
     *
     * @param s  the spline to parametrize
     * @param t0 starting percentage of spline to parametrize
     * @param t1 ending percentage of spline to parametrize
     * @return list of Pose2dWithCurvature that approximates the original spline
     */
    public static List<Pose2dWithCurvature> parameterizeSpline(Spline s, double maxDx, double maxDy, double
            maxDTheta, double t0, double t1) {
        List<Pose2dWithCurvature> rv = new ArrayList<>();
        rv.add(s.getPose2dWithCurvature(0.0));
        double dt = (t1 - t0);
        for (double t = 0; t < t1; t += dt / kMinSampleSize) {
            getSegmentArc(s, rv, t, t + dt / kMinSampleSize, maxDx, maxDy, maxDTheta);
        }
        return rv;
    }

    /**
     * Convenience function to parametrize a spline from t 0 to 1
     */
    public static List<Pose2dWithCurvature> parameterizeSpline(Spline s) {
        return parameterizeSpline(s, kMaxDX, kMaxDY, kMaxDTheta, 0.0, 1.0);
    }

    public static List<Pose2dWithCurvature> parameterizeSpline(Spline s, double maxDx, double maxDy, double maxDTheta) {
        return parameterizeSpline(s, maxDx, maxDy, maxDTheta, 0.0, 1.0);
    }

    public static List<Pose2dWithCurvature> parameterizeSplines(List<Spline> splines) {
        return parameterizeSplines(splines, kMaxDX, kMaxDY, kMaxDTheta);
    }

    public static List<Pose2dWithCurvature> parameterizeSplines(List<? extends Spline> splines, double maxDx, double maxDy,
                                                                double maxDTheta) {
        List<Pose2dWithCurvature> rv = new ArrayList<>();
        if (splines.isEmpty()) return rv;
        rv.add(splines.get(0).getPose2dWithCurvature(0.0));
        for (final Spline s : splines) {
            List<Pose2dWithCurvature> samples = parameterizeSpline(s, maxDx, maxDy, maxDTheta);
            samples.remove(0);
            rv.addAll(samples);
        }
        return rv;
    }

    private static void getSegmentArc(Spline s, List<Pose2dWithCurvature> rv, double t0, double t1, double maxDx,
                                      double maxDy,
                                      double maxDTheta) {
        Translation2d p0 = s.getPoint(t0);
        Translation2d p1 = s.getPoint(t1);
        Rotation2d r0 = s.getHeading(t0);
        Rotation2d r1 = s.getHeading(t1);
        Pose2d transformation = new Pose2d(new Translation2d(p0, p1).rotateBy(r0.inverse()), r1.rotateBy(r0.inverse()));
        Twist2d twist = Pose2d.log(transformation);
        if (twist.dy > maxDy || twist.dx > maxDx || twist.dtheta > maxDTheta) {
            getSegmentArc(s, rv, t0, (t0 + t1) / 2, maxDx, maxDy, maxDTheta);
            getSegmentArc(s, rv, (t0 + t1) / 2, t1, maxDx, maxDy, maxDTheta);
        } else {
            rv.add(s.getPose2dWithCurvature(t1));
        }
    }

}
