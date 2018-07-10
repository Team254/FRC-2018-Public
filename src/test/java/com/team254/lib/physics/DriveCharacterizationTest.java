package com.team254.lib.physics;

import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class DriveCharacterizationTest {
    public static final double kTestEpsilon = 1e-4;

    @Test
    public void test() {
        final double ks = 0.75; //Math.random();
        final double kv = 0.2; //Math.random();
        final double ka = 0.15; //Math.random();

        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        // generate velocity data points
        for (double v = 0; v < 1.0; v += 0.01) {
            velocityData.add(new DriveCharacterization.VelocityDataPoint(Math.max(0.0, (v - ks) / kv), v));
        }

        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();
        double v, a;
        v = 0;
        // generate acceleration data points
        for (int i = 0; i < 1000; ++i) {
            a = Math.max(0.0, 6.0 - kv * v - ks) / ka;
            v += a * kTestEpsilon;
            accelerationData.add(new DriveCharacterization.AccelerationDataPoint(v, 6.0, a));
        }

        DriveCharacterization.CharacterizationConstants driveConstants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        assertEquals(driveConstants.ks, ks, kTestEpsilon);
        assertEquals(driveConstants.kv, kv, kTestEpsilon);
        assertEquals(driveConstants.ka, ka, kTestEpsilon);
    }
}
