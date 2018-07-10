package com.team254.lib.physics;

import com.team254.lib.util.Util;

/**
 * Model of a DC motor rotating a shaft.  All parameters refer to the output (e.g. should already consider gearing
 * and efficiency losses).  The motor is assumed to be symmetric forward/reverse.
 */
public class DCMotorTransmission {
    // TODO add electrical constants?  (e.g. current)

    // All units must be SI!
    protected final double speed_per_volt_;  // rad/s per V (no load)
    protected final double torque_per_volt_;  // N m per V (stall)
    protected final double friction_voltage_;  // V

    public DCMotorTransmission(final double speed_per_volt,
                               final double torque_per_volt,
                               final double friction_voltage) {
        speed_per_volt_ = speed_per_volt;
        torque_per_volt_ = torque_per_volt;
        friction_voltage_ = friction_voltage;
    }

    public double speed_per_volt() {
        return speed_per_volt_;
    }

    public double torque_per_volt() {
        return torque_per_volt_;
    }

    public double friction_voltage() {
        return friction_voltage_;
    }

    public double free_speed_at_voltage(final double voltage) {
        if (voltage > Util.kEpsilon) {
            return Math.max(0.0, voltage - friction_voltage()) * speed_per_volt();
        } else if (voltage < Util.kEpsilon) {
            return Math.min(0.0, voltage + friction_voltage()) * speed_per_volt();
        } else {
            return 0.0;
        }
    }

    public double getTorqueForVoltage(final double output_speed, final double voltage) {
        double effective_voltage = voltage;
        if (output_speed > Util.kEpsilon) {
            // Forward motion, rolling friction.
            effective_voltage -= friction_voltage();
        } else if (output_speed < -Util.kEpsilon) {
            // Reverse motion, rolling friction.
            effective_voltage += friction_voltage();
        } else if (voltage > Util.kEpsilon) {
            // System is static, forward torque.
            effective_voltage = Math.max(0.0, voltage - friction_voltage());
        } else if (voltage < -Util.kEpsilon) {
            // System is static, reverse torque.
            effective_voltage = Math.min(0.0, voltage + friction_voltage());
        } else {
            // System is idle.
            return 0.0;
        }
        return torque_per_volt() * (-output_speed / speed_per_volt() + effective_voltage);
    }

    public double getVoltageForTorque(final double output_speed, final double torque) {
        double friction_voltage;
        if (output_speed > Util.kEpsilon) {
            // Forward motion, rolling friction.
            friction_voltage = friction_voltage();
        } else if (output_speed < -Util.kEpsilon) {
            // Reverse motion, rolling friction.
            friction_voltage = -friction_voltage();
        } else if (torque > Util.kEpsilon) {
            // System is static, forward torque.
            friction_voltage = friction_voltage();
        } else if (torque < -Util.kEpsilon) {
            // System is static, reverse torque.
            friction_voltage = -friction_voltage();
        } else {
            // System is idle.
            return 0.0;
        }
        return torque / torque_per_volt() + output_speed / speed_per_volt() + friction_voltage;
    }
}
