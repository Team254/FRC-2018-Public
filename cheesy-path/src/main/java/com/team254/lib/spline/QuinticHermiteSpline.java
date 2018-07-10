package com.team254.lib.spline;

import java.util.List;

import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;

public class QuinticHermiteSpline extends Spline {
    private static final double kEpsilon = 1e-5;
    private static final double kStepSize = 1.0;
    private static final double kMinDelta = 0.001;
    private static final int kSamples = 100;
    private static final int kMaxIterations = 100;

    private double x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1;
    private double ax, bx, cx, dx, ex, fx, ay, by, cy, dy, ey, fy;

    /**
     * @param p0 The starting pose of the spline
     * @param p1 The ending pose of the spline
     * @param ref ReferenceFrame of this spline
     */
    public QuinticHermiteSpline(Pose2d p0, Pose2d p1) {
        double scale = 1.2 * p0.getTranslation().distance(p1.getTranslation());
        x0 = p0.getTranslation().x();
        x1 = p1.getTranslation().x();
        dx0 = p0.getRotation().cos() * scale;
        dx1 = p1.getRotation().cos() * scale;
        ddx0 = 0;
        ddx1 = 0;
        y0 = p0.getTranslation().y();
        y1 = p1.getTranslation().y();
        dy0 = p0.getRotation().sin() * scale;
        dy1 = p1.getRotation().sin() * scale;
        ddy0 = 0;
        ddy1 = 0;

        computeCoefficients();
    }

    /**
     * Used by the curvature optimization function
     */
    private QuinticHermiteSpline(double x0, double x1, double dx0, double dx1, double ddx0, double ddx1,
                                 double y0, double y1, double dy0, double dy1, double ddy0, double ddy1) {
        this.x0 = x0;
        this.x1 = x1;
        this.dx0 = dx0;
        this.dx1 = dx1;
        this.ddx0 = ddx0;
        this.ddx1 = ddx1;

        this.y0 = y0;
        this.y1 = y1;
        this.dy0 = dy0;
        this.dy1 = dy1;
        this.ddy0 = ddy0;
        this.ddy1 = ddy1;

        computeCoefficients();
    }

    /**
     * Re-arranges the spline into an at^5 + bt^4 + ... + f form for simpler computations
     */
    private void computeCoefficients() {
        ax = -6 * x0 - 3 * dx0 - 0.5 * ddx0 + 0.5 * ddx1 - 3 * dx1 + 6 * x1;
        bx = 15 * x0 + 8 * dx0 + 1.5 * ddx0 - ddx1 + 7 * dx1 - 15 * x1;
        cx = -10 * x0 - 6 * dx0 - 1.5 * ddx0 + 0.5 * ddx1 - 4 * dx1 + 10 * x1;
        dx = 0.5 * ddx0;
        ex = dx0;
        fx = x0;

        ay = -6 * y0 - 3 * dy0 - 0.5 * ddy0 + 0.5 * ddy1 - 3 * dy1 + 6 * y1;
        by = 15 * y0 + 8 * dy0 + 1.5 * ddy0 - ddy1 + 7 * dy1 - 15 * y1;
        cy = -10 * y0 - 6 * dy0 - 1.5 * ddy0 + 0.5 * ddy1 - 4 * dy1 + 10 * y1;
        dy = 0.5 * ddy0;
        ey = dy0;
        fy = y0;
    }

    public Pose2d getStartPose() {
        return new Pose2d(
                new Translation2d(x0, y0),
                new Rotation2d(dx0, dy0, true)
        );
    }

    public Pose2d getEndPose() {
        return new Pose2d(
                new Translation2d(x1, y1),
                new Rotation2d(dx1, dy1, true)
        );
    }

    /**
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    @Override
    public Translation2d getPoint(double t) {
        double x = ax * t * t * t * t * t + bx * t * t * t * t + cx * t * t * t + dx * t * t + ex * t + fx;
        double y = ay * t * t * t * t * t + by * t * t * t * t + cy * t * t * t + dy * t * t + ey * t + fy;
        return new Translation2d(x, y);
    }

    private double dx(double t) {
        return 5 * ax * t * t * t * t + 4 * bx * t * t * t + 3 * cx * t * t + 2 * dx * t + ex;
    }

    private double dy(double t) {
        return 5 * ay * t * t * t * t + 4 * by * t * t * t + 3 * cy * t * t + 2 * dy * t + ey;
    }

    private double ddx(double t) {
        return 20 * ax * t * t * t + 12 * bx * t * t + 6 * cx * t + 2 * dx;
    }

    private double ddy(double t) {
        return 20 * ay * t * t * t + 12 * by * t * t + 6 * cy * t + 2 * dy;
    }

    private double dddx(double t) {
        return 60 * ax * t * t + 24 * bx * t + 6 * cx;
    }

    private double dddy(double t) {
        return 60 * ay * t * t + 24 * by * t + 6 * cy;
    }

    @Override
    public double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }

    @Override
    public double getCurvature(double t) {
        return (dx(t) * ddy(t) - ddx(t) * dy(t)) / ((dx(t) * dx(t) + dy(t) * dy(t)) * Math.sqrt((dx(t) * dx(t) + dy
                (t) * dy(t))));
    }

    @Override
    public  double getDCurvature(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t)*dddy(t) - dddx(t)*dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }

    private double dCurvature2(double t) {
        double dx2dy2 = (dx(t) * dx(t) + dy(t) * dy(t));
        double num = (dx(t)*dddy(t) - dddx(t)*dy(t)) * dx2dy2 - 3 * (dx(t) * ddy(t) - ddx(t) * dy(t)) * (dx(t) * ddx(t) + dy(t) * ddy(t));
        return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
    }

    @Override
    public Rotation2d getHeading(double t) {
        return new Rotation2d(dx(t), dy(t), true);
    }

    /**
     * @return integral of dCurvature^2 over the length of the spline
     */
    private double sumDCurvature2() {
        double dt = 1.0 / kSamples;
        double sum = 0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * dCurvature2(t));
        }
        return sum;
    }

    /**
     * @return integral of dCurvature^2 over the length of multiple splines
     */
    public static double sumDCurvature2(List<QuinticHermiteSpline> splines) {
        double sum = 0;
        for (QuinticHermiteSpline s : splines) {
            sum += s.sumDCurvature2();
        }
        return sum;
    }

    /**
     * Makes optimization code a little more readable
     */
    private static class ControlPoint {
        private double ddx, ddy;
    }

    /**
     * Finds the optimal second derivative values for a set of splines to reduce the sum of the change in curvature
     * squared over the path
     *
     * @param splines the list of splines to optimize
     * @return the final sumDCurvature2
     */
    public static double optimizeSpline(List<QuinticHermiteSpline> splines) {
        int count = 0;
        double prev = sumDCurvature2(splines);
        while (count < kMaxIterations) {
            runOptimizationIteration(splines);
            double current = sumDCurvature2(splines);
            if (prev - current < kMinDelta)
                return current;
            prev = current;
            count++;
        }
        return prev;
    }


    /**
     * Runs a single optimization iteration
     */
    private static void runOptimizationIteration(List<QuinticHermiteSpline> splines) {
        //can't optimize anything with less than 2 splines
        if (splines.size() <= 1) {
            return;
        }

        ControlPoint[] controlPoints = new ControlPoint[splines.size() - 1];
        double magnitude = 0;

        for (int i = 0; i < splines.size() - 1; ++i) {
            //don't try to optimize colinear points
            if(splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            double original = sumDCurvature2(splines);
            QuinticHermiteSpline temp, temp1;

            temp = splines.get(i);
            temp1 = splines.get(i + 1);
            controlPoints[i] = new ControlPoint(); //holds the gradient at a control point

            //calculate partial derivatives of sumDCurvature2
            splines.set(i, new QuinticHermiteSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0, temp.ddx1 +
                    kEpsilon, temp.y0, temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1));
            splines.set(i + 1, new QuinticHermiteSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0 +
                    kEpsilon, temp1.ddx1, temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0, temp1.ddy1));
            controlPoints[i].ddx = (sumDCurvature2(splines) - original) / kEpsilon;
            splines.set(i, new QuinticHermiteSpline(temp.x0, temp.x1, temp.dx0, temp.dx1, temp.ddx0, temp.ddx1, temp
                    .y0, temp.y1, temp.dy0, temp.dy1, temp.ddy0, temp.ddy1 + kEpsilon));
            splines.set(i + 1, new QuinticHermiteSpline(temp1.x0, temp1.x1, temp1.dx0, temp1.dx1, temp1.ddx0,
                    temp1.ddx1, temp1.y0, temp1.y1, temp1.dy0, temp1.dy1, temp1.ddy0 + kEpsilon, temp1.ddy1));
            controlPoints[i].ddy = (sumDCurvature2(splines) - original) / kEpsilon;

            splines.set(i, temp);
            splines.set(i + 1, temp1);
            magnitude += controlPoints[i].ddx * controlPoints[i].ddx + controlPoints[i].ddy * controlPoints[i].ddy;
        }

        magnitude = Math.sqrt(magnitude);

        //minimize along the direction of the gradient
        //first calculate 3 points along the direction of the gradient
        Translation2d p1, p2, p3;
        p2 = new Translation2d(0, sumDCurvature2(splines)); //middle point is at the current location

        for (int i = 0; i < splines.size() - 1; ++i) { //first point is offset from the middle location by -stepSize
            if(splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            //normalize to step size
            controlPoints[i].ddx *= kStepSize / magnitude;
            controlPoints[i].ddy *= kStepSize / magnitude;

            //move opposite the gradient by step size amount
            splines.get(i).ddx1 -= controlPoints[i].ddx;
            splines.get(i).ddy1 -= controlPoints[i].ddy;
            splines.get(i + 1).ddx0 -= controlPoints[i].ddx;
            splines.get(i + 1).ddy0 -= controlPoints[i].ddy;

            //recompute the spline's coefficients to account for new second derivatives
            splines.get(i).computeCoefficients();
            splines.get(i + 1).computeCoefficients();
        }
        p1 = new Translation2d(-kStepSize, sumDCurvature2(splines));

        for (int i = 0; i < splines.size() - 1; ++i) { //last point is offset from the middle location by +stepSize
            if(splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            //move along the gradient by 2 times the step size amount (to return to original location and move by 1
            // step)
            splines.get(i).ddx1 += 2 * controlPoints[i].ddx;
            splines.get(i).ddy1 += 2 * controlPoints[i].ddy;
            splines.get(i + 1).ddx0 += 2 * controlPoints[i].ddx;
            splines.get(i + 1).ddy0 += 2 * controlPoints[i].ddy;

            //recompute the spline's coefficients to account for new second derivatives
            splines.get(i).computeCoefficients();
            splines.get(i + 1).computeCoefficients();
        }

        p3 = new Translation2d(kStepSize, sumDCurvature2(splines));

        double stepSize = fitParabola(p1, p2, p3); //approximate step size to minimize sumDCurvature2 along the gradient

        for (int i = 0; i < splines.size() - 1; ++i) {
            if(splines.get(i).getStartPose().isColinear(splines.get(i + 1).getStartPose()) || splines.get(i).getEndPose().isColinear(splines.get(i + 1).getEndPose())) {
                continue;
            }
            //move by the step size calculated by the parabola fit (+1 to offset for the final transformation to find
            // p3)
            controlPoints[i].ddx *= 1 + stepSize / kStepSize;
            controlPoints[i].ddy *= 1 + stepSize / kStepSize;

            splines.get(i).ddx1 += controlPoints[i].ddx;
            splines.get(i).ddy1 += controlPoints[i].ddy;
            splines.get(i + 1).ddx0 += controlPoints[i].ddx;
            splines.get(i + 1).ddy0 += controlPoints[i].ddy;

            //recompute the spline's coefficients to account for new second derivatives
            splines.get(i).computeCoefficients();
            splines.get(i + 1).computeCoefficients();
        }
    }

    /**
     * fits a parabola to 3 points
     *
     * @return the x coordinate of the vertex of the parabola
     */
    private static double fitParabola(Translation2d p1, Translation2d p2, Translation2d p3) {
        double A = (p3.x() * (p2.y() - p1.y()) + p2.x() * (p1.y() - p3.y()) + p1.x() * (p3.y() - p2.y()));
        double B = (p3.x() * p3.x() * (p1.y() - p2.y()) + p2.x() * p2.x() * (p3.y() - p1.y()) + p1.x() * p1.x() *
                (p2.y() - p3.y()));
        return -B / (2 * A);
    }
}