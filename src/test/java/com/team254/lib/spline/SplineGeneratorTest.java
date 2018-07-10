package com.team254.lib.spline;

import com.team254.lib.geometry.*;
import com.team254.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class SplineGeneratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    @Test
    public void test() {
        // Create the test spline
        Pose2d p1 = new Pose2d(new Translation2d(0, 0), new Rotation2d());
        Pose2d p2 = new Pose2d(new Translation2d(15, 10), new Rotation2d(1, -5, true));
        Spline s = new QuinticHermiteSpline(p1, p2);

        List<Pose2dWithCurvature> samples = SplineGenerator.parameterizeSpline(s);

        double arclength = 0;
        Pose2dWithCurvature cur_pose = samples.get(0);
        for (Pose2dWithCurvature sample : samples) {
            final Twist2d t = Pose2d.log(cur_pose.getPose().inverse().transformBy(sample.getPose()));
            arclength += t.dx;
            cur_pose = sample;
        }

        assertEquals(cur_pose.getTranslation().x(), 15.0, kTestEpsilon);
        assertEquals(cur_pose.getTranslation().y(), 10.0, kTestEpsilon);
        assertEquals(cur_pose.getRotation().getDegrees(), -78.69006752597981, kTestEpsilon);
        assertEquals(arclength, 23.17291953186379, kTestEpsilon);
    }
}
