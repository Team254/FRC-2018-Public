package com.team254.lib.physics;

import com.team254.lib.util.PolynomialRegression;
import com.team254.lib.util.Util;

import java.util.List;

public class DriveCharacterization {
    public static class CharacterizationConstants {
        public double ks; //voltage needed to break static friction
        public double kv;
        public double ka;
    }

    public static class VelocityDataPoint {
        public final double velocity;
        public final double power;

        public VelocityDataPoint(double velocity, double power) {
            this.velocity = velocity;
            this.power = power;
        }
    }

    public static class AccelerationDataPoint {
        public final double velocity;
        public final double power;
        public final double acceleration;

        public AccelerationDataPoint(double velocity, double power, double acceleration) {
            this.velocity = velocity;
            this.power = power;
            this.acceleration = acceleration;
        }
    }

    public static class CurvatureDataPoint {
        public final double linear_velocity;
        public final double angular_velocity;
        public final double left_voltage;
        public final double right_voltage;

        public CurvatureDataPoint(double linear_velocity, double angular_velocity, double left_voltage, double right_voltage) {
            this.linear_velocity = linear_velocity;
            this.angular_velocity = angular_velocity;
            this.left_voltage = left_voltage;
            this.right_voltage = right_voltage;
        }
    }

    public static CharacterizationConstants characterizeDrive(List<VelocityDataPoint> velocityData, List<AccelerationDataPoint> accelerationData) {
        CharacterizationConstants rv = getVelocityCharacterization(getVelocityData(velocityData));
        getAccelerationCharacterization(getAccelerationData(accelerationData, rv), rv);
        return rv;
    }

    private static CharacterizationConstants getVelocityCharacterization(double[][] points) {
        CharacterizationConstants constants = new CharacterizationConstants();
        if (points == null) {
            return constants;
        }
        PolynomialRegression p = new PolynomialRegression(points, 1);
        System.out.println("r^2: " + p.R2());
        constants.ks = p.beta(0);
        constants.kv = p.beta(1);
        return constants;
    }

    private static CharacterizationConstants getAccelerationCharacterization(double[][] points, CharacterizationConstants velocityChacterization) {
        if (points == null) {
            return velocityChacterization;
        }

        PolynomialRegression p = new PolynomialRegression(points, 1);
        System.out.println("r^2: " + p.R2());
        velocityChacterization.ka = p.beta(1);
        return velocityChacterization;
    }

    /**
     * removes data points with a velocity of zero to get a better line fit
     */
    private static double[][] getVelocityData(List<VelocityDataPoint> input) {
        double[][] output = null;
        int startTrim = 0;
        for (int i = 0; i < input.size(); ++i) {
            if (input.get(i).velocity > Util.kEpsilon) {
                if (output == null) {
                    output = new double[input.size() - i][2];
                    startTrim = i;
                }
                output[i - startTrim][0] = input.get(i).velocity;
                output[i - startTrim][1] = input.get(i).power;
            }
        }
        return output;
    }

    private static double[][] getAccelerationData(List<AccelerationDataPoint> input, CharacterizationConstants constants) {
        double[][] output = new double[input.size()][2];
        for (int i = 0; i < input.size(); ++i) {
            output[i][0] = input.get(i).acceleration;
            output[i][1] = input.get(i).power - constants.kv * input.get(i).velocity - constants.ks;
        }
        return output;
    }
}
