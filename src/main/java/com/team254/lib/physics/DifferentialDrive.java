package com.team254.lib.physics;

import com.team254.lib.util.CSVWritable;
import com.team254.lib.util.Util;

import java.text.DecimalFormat;
import java.util.Arrays;

/**
 * Dynamic model a differential drive robot.  Note: to simplify things, this math assumes the center of mass is
 * coincident with the kinematic center of rotation (e.g. midpoint of the center axle).
 */
public class DifferentialDrive {
    // All units must be SI!

    // Equivalent mass when accelerating purely linearly, in kg.
    // This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
    // Measure by doing drivetrain acceleration characterization in a straight line.
    protected final double mass_;

    // Equivalent moment of inertia when accelerating purely angularly, in kg*m^2.
    // This is "equivalent" in that it also absorbs the effects of drivetrain inertia.
    // Measure by doing drivetrain acceleration characterization while turning in place.
    protected final double moi_;

    // Drag torque (proportional to angular velocity) that resists turning, in N*m/rad/s
    // Empirical testing of our drivebase showed that there was an unexplained loss in torque ~proportional to angular
    // velocity, likely due to scrub of wheels.
    // NOTE: this may not be a purely linear term, and we have done limited testing, but this factor helps our model to
    // better match reality.  For future seasons, we should investigate what's going on here...
    protected final double angular_drag_;

    // Self-explanatory.  Measure by rolling the robot a known distance and counting encoder ticks.
    protected final double wheel_radius_;  // m

    // "Effective" kinematic wheelbase radius.  Might be larger than theoretical to compensate for skid steer.  Measure
    // by turning the robot in place several times and figuring out what the equivalent wheelbase radius is.
    protected final double effective_wheelbase_radius_;  // m

    // Transmissions for both sides of the drive.
    protected final DCMotorTransmission left_transmission_;
    protected final DCMotorTransmission right_transmission_;

    public DifferentialDrive(final double mass,
                             final double moi,
                             final double angular_drag,
                             final double wheel_radius,
                             final double effective_wheelbase_radius,
                             final DCMotorTransmission left_transmission,
                             final DCMotorTransmission right_transmission) {
        mass_ = mass;
        moi_ = moi;
        angular_drag_ = angular_drag;
        wheel_radius_ = wheel_radius;
        effective_wheelbase_radius_ = effective_wheelbase_radius;
        left_transmission_ = left_transmission;
        right_transmission_ = right_transmission;
    }

    public double mass() {
        return mass_;
    }

    public double moi() {
        return moi_;
    }

    public double wheel_radius() {
        return wheel_radius_;
    }

    public double effective_wheelbase_radius() {
        return effective_wheelbase_radius_;
    }

    public DCMotorTransmission left_transmission() {
        return left_transmission_;
    }

    public DCMotorTransmission right_transmission() {
        return right_transmission_;
    }

    // Input/demand could be either velocity or acceleration...the math is the same.
    public ChassisState solveForwardKinematics(final WheelState wheel_motion) {
        ChassisState chassis_motion = new ChassisState();
        chassis_motion.linear = wheel_radius_ * (wheel_motion.right + wheel_motion.left) / 2.0;
        chassis_motion.angular = wheel_radius_ * (wheel_motion.right - wheel_motion.left) / (2.0 *
                effective_wheelbase_radius_);
        return chassis_motion;
    }

    // Input/output could be either velocity or acceleration...the math is the same.
    public WheelState solveInverseKinematics(final ChassisState chassis_motion) {
        WheelState wheel_motion = new WheelState();
        wheel_motion.left = (chassis_motion.linear - effective_wheelbase_radius_ * chassis_motion.angular) /
                wheel_radius_;
        wheel_motion.right = (chassis_motion.linear + effective_wheelbase_radius_ * chassis_motion.angular) /
                wheel_radius_;
        return wheel_motion;
    }

    // Solve for torques and accelerations.
    public DriveDynamics solveForwardDynamics(final ChassisState chassis_velocity, final WheelState voltage) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
        dynamics.chassis_velocity = chassis_velocity;
        dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
        if (Double.isNaN(dynamics.curvature)) dynamics.curvature = 0.0;
        dynamics.voltage = voltage;
        solveForwardDynamics(dynamics);
        return dynamics;
    }

    public DriveDynamics solveForwardDynamics(final WheelState wheel_velocity, final WheelState voltage) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.wheel_velocity = wheel_velocity;
        dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
        dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
        if (Double.isNaN(dynamics.curvature)) dynamics.curvature = 0.0;
        dynamics.voltage = voltage;
        solveForwardDynamics(dynamics);
        return dynamics;
    }

    // Assumptions about dynamics: velocities and voltages provided.
    public void solveForwardDynamics(DriveDynamics dynamics) {
        final boolean left_stationary = Util.epsilonEquals(dynamics.wheel_velocity.left, 0.0) && Math.abs(dynamics
                .voltage.left) < left_transmission_.friction_voltage();
        final boolean right_stationary = Util.epsilonEquals(dynamics.wheel_velocity.right, 0.0) && Math.abs(dynamics
                .voltage.right) < right_transmission_.friction_voltage();
        if (left_stationary && right_stationary) {
            // Neither side breaks static friction, so we remain stationary.
            dynamics.wheel_torque.left = dynamics.wheel_torque.right = 0.0;
            dynamics.chassis_acceleration.linear = dynamics.chassis_acceleration.angular = 0.0;
            dynamics.wheel_acceleration.left = dynamics.wheel_acceleration.right = 0.0;
            dynamics.dcurvature = 0.0;
            return;
        }

        // Solve for motor torques generated on each side.
        dynamics.wheel_torque.left = left_transmission_.getTorqueForVoltage(dynamics.wheel_velocity.left, dynamics
                .voltage.left);
        dynamics.wheel_torque.right = right_transmission_.getTorqueForVoltage(dynamics.wheel_velocity.right, dynamics
                .voltage.right);

        // Add forces and torques about the center of mass.
        dynamics.chassis_acceleration.linear = (dynamics.wheel_torque.right + dynamics.wheel_torque.left) /
                (wheel_radius_ * mass_);
        // (Tr - Tl) / r_w * r_wb - drag * w = I * angular_accel
        dynamics.chassis_acceleration.angular = effective_wheelbase_radius_ * (dynamics.wheel_torque.right - dynamics
                .wheel_torque.left) / (wheel_radius_ * moi_) - dynamics.chassis_velocity.angular * angular_drag_ / moi_;

        // Solve for change in curvature from angular acceleration.
        // total angular accel = linear_accel * curvature + v^2 * dcurvature
        dynamics.dcurvature = (dynamics.chassis_acceleration.angular - dynamics.chassis_acceleration.linear * dynamics.curvature) /
                (dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear);
        if (Double.isNaN(dynamics.dcurvature)) dynamics.dcurvature = 0.0;

        // Resolve chassis accelerations to each wheel.
        dynamics.wheel_acceleration.left = dynamics.chassis_acceleration.linear - dynamics.chassis_acceleration
                .angular * effective_wheelbase_radius_;
        dynamics.wheel_acceleration.right = dynamics.chassis_acceleration.linear + dynamics.chassis_acceleration
                .angular * effective_wheelbase_radius_;
    }

    // Solve for torques and voltages.
    public DriveDynamics solveInverseDynamics(final ChassisState chassis_velocity, final ChassisState
            chassis_acceleration) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.chassis_velocity = chassis_velocity;
        dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
        if (Double.isNaN(dynamics.curvature)) dynamics.curvature = 0.0;
        dynamics.chassis_acceleration = chassis_acceleration;
        dynamics.dcurvature = (dynamics.chassis_acceleration.angular - dynamics.chassis_acceleration.linear * dynamics.curvature) /
                (dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear);
        if (Double.isNaN(dynamics.dcurvature)) dynamics.dcurvature = 0.0;
        dynamics.wheel_velocity = solveInverseKinematics(chassis_velocity);
        dynamics.wheel_acceleration = solveInverseKinematics(chassis_acceleration);
        solveInverseDynamics(dynamics);
        return dynamics;
    }

    public DriveDynamics solveInverseDynamics(final WheelState wheel_velocity, final WheelState wheel_acceleration) {
        DriveDynamics dynamics = new DriveDynamics();
        dynamics.chassis_velocity = solveForwardKinematics(wheel_velocity);
        dynamics.curvature = dynamics.chassis_velocity.angular / dynamics.chassis_velocity.linear;
        if (Double.isNaN(dynamics.curvature)) dynamics.curvature = 0.0;
        dynamics.chassis_acceleration = solveForwardKinematics(wheel_acceleration);
        dynamics.dcurvature = (dynamics.chassis_acceleration.angular - dynamics.chassis_acceleration.linear * dynamics.curvature) /
                (dynamics.chassis_velocity.linear * dynamics.chassis_velocity.linear);
        if (Double.isNaN(dynamics.dcurvature)) dynamics.dcurvature = 0.0;
        dynamics.wheel_velocity = wheel_velocity;
        dynamics.wheel_acceleration = wheel_acceleration;
        solveInverseDynamics(dynamics);
        return dynamics;
    }

    // Assumptions about dynamics: velocities and accelerations provided, curvature and dcurvature computed.
    public void solveInverseDynamics(DriveDynamics dynamics) {
        // Determine the necessary torques on the left and right wheels to produce the desired wheel accelerations.
        dynamics.wheel_torque.left = wheel_radius_ / 2.0 * (dynamics.chassis_acceleration.linear * mass_ -
                dynamics.chassis_acceleration.angular * moi_ / effective_wheelbase_radius_ -
                dynamics.chassis_velocity.angular * angular_drag_ / effective_wheelbase_radius_);
        dynamics.wheel_torque.right = wheel_radius_ / 2.0 * (dynamics.chassis_acceleration.linear * mass_ +
                dynamics.chassis_acceleration.angular * moi_ / effective_wheelbase_radius_ +
                dynamics.chassis_velocity.angular * angular_drag_ / effective_wheelbase_radius_);

        // Solve for input voltages.
        dynamics.voltage.left = left_transmission_.getVoltageForTorque(dynamics.wheel_velocity.left, dynamics
                .wheel_torque.left);
        dynamics.voltage.right = right_transmission_.getVoltageForTorque(dynamics.wheel_velocity.right, dynamics
                .wheel_torque.right);
    }

    public double getMaxAbsVelocity(double curvature, /*double dcurvature, */double max_abs_voltage) {
        // Alternative implementation:
        // (Tr - Tl) * r_wb / r_w = I * v^2 * dk
        // (Tr + Tl) / r_w = 0
        // T = Tr = -Tl
        // 2T * r_wb / r_w = I*v^2*dk
        // T = 2*I*v^2*dk*r_w/r_wb
        // T = kt*(-vR/kv + V) = -kt*(-vL/vmax + V)
        // Vr = v * (1 + k*r_wb)
        // 0 = 2*I*dk*r_w/r_wb * v^2 + kt * ((1 + k*r_wb) * v / kv) - kt * V
        // solve using quadratic formula?
        // -b +/- sqrt(b^2 - 4*a*c) / (2a)

        // k = w / v
        // v = r_w*(wr + wl) / 2
        // w = r_w*(wr - wl) / (2 * r_wb)
        // Plug in max_abs_voltage for each wheel.
        final double left_speed_at_max_voltage = left_transmission_.free_speed_at_voltage(max_abs_voltage);
        final double right_speed_at_max_voltage = right_transmission_.free_speed_at_voltage(max_abs_voltage);
        if (Util.epsilonEquals(curvature, 0.0)) {
            return wheel_radius_ * Math.min(left_speed_at_max_voltage, right_speed_at_max_voltage);
        }
        if (Double.isInfinite(curvature)) {
            // Turn in place.  Return value meaning becomes angular velocity.
            final double wheel_speed = Math.min(left_speed_at_max_voltage, right_speed_at_max_voltage);
            return Math.signum(curvature) * wheel_radius_ * wheel_speed / effective_wheelbase_radius_;
        }

        final double right_speed_if_left_max = left_speed_at_max_voltage * (effective_wheelbase_radius_ * curvature +
                1.0) / (1.0 - effective_wheelbase_radius_ * curvature);
        if (Math.abs(right_speed_if_left_max) <= right_speed_at_max_voltage + Util.kEpsilon) {
            // Left max is active constraint.
            return wheel_radius_ * (left_speed_at_max_voltage + right_speed_if_left_max) / 2.0;
        }
        final double left_speed_if_right_max = right_speed_at_max_voltage * (1.0 - effective_wheelbase_radius_ *
                curvature) / (1.0 + effective_wheelbase_radius_ * curvature);
        // Right at max is active constraint.
        return wheel_radius_ * (right_speed_at_max_voltage + left_speed_if_right_max) / 2.0;
    }

    public static class MinMax {
        public double min;
        public double max;
    }

    // Curvature is redundant here in the case that chassis_velocity is not purely angular.  It is the responsibility of
    // the caller to ensure that curvature = angular vel / linear vel in these cases.
    public MinMax getMinMaxAcceleration(final ChassisState chassis_velocity, double curvature, /*double dcurvature,*/ double
            max_abs_voltage) {
        MinMax result = new MinMax();
        final WheelState wheel_velocities = solveInverseKinematics(chassis_velocity);
        result.min = Double.POSITIVE_INFINITY;
        result.max = Double.NEGATIVE_INFINITY;

        // Math:
        // (Tl + Tr) / r_w = m*a
        // (Tr - Tl) / r_w * r_wb - drag*w = i*(a * k + v^2 * dk)

        // 2 equations, 2 unknowns.
        // Solve for a and (Tl|Tr)

        final double linear_term = Double.isInfinite(curvature) ? 0.0 : mass_ * effective_wheelbase_radius_;
        final double angular_term = Double.isInfinite(curvature) ? moi_ : moi_ * curvature;

        final double drag_torque = chassis_velocity.angular * angular_drag_;

        // Check all four cases and record the min and max valid accelerations.
        for (boolean left : Arrays.asList(false, true)) {
            for (double sign : Arrays.asList(1.0, -1.0)) {
                final DCMotorTransmission fixed_transmission = left ? left_transmission_ : right_transmission_;
                final DCMotorTransmission variable_transmission = left ? right_transmission_ : left_transmission_;
                final double fixed_torque = fixed_transmission.getTorqueForVoltage(wheel_velocities.get(left), sign *
                        max_abs_voltage);
                double variable_torque = 0.0;
                // NOTE: variable_torque is wrong.  Units don't work out correctly.  We made a math error somewhere...
                // Leaving this "as is" for code release so as not to be disingenuous, but this whole function needs
                // revisiting in the future...
                if (left) {
                    variable_torque = ((/*-moi_ * chassis_velocity.linear * chassis_velocity.linear * dcurvature*/ -drag_torque) * mass_ * wheel_radius_ + fixed_torque *
                            (linear_term + angular_term)) / (linear_term - angular_term);
                } else {
                    variable_torque = ((/*moi_ * chassis_velocity.linear * chassis_velocity.linear * dcurvature*/ +drag_torque) * mass_ * wheel_radius_ + fixed_torque *
                            (linear_term - angular_term)) / (linear_term + angular_term);
                }
                final double variable_voltage = variable_transmission.getVoltageForTorque(wheel_velocities.get(!left), variable_torque);
                if (Math.abs(variable_voltage) <= max_abs_voltage + Util.kEpsilon) {
                    double accel = 0.0;
                    if (Double.isInfinite(curvature)) {
                        accel = (left ? -1.0 : 1.0) * (fixed_torque - variable_torque) * effective_wheelbase_radius_
                                / (moi_ * wheel_radius_) - drag_torque / moi_ /*- chassis_velocity.linear * chassis_velocity.linear * dcurvature*/;
                    } else {
                        accel = (fixed_torque + variable_torque) / (mass_ * wheel_radius_);
                    }
                    result.min = Math.min(result.min, accel);
                    result.max = Math.max(result.max, accel);
                }
            }
        }
        return result;
    }

    // Can refer to velocity or acceleration depending on context.
    public static class ChassisState {
        public double linear;
        public double angular;

        public ChassisState(double linear, double angular) {
            this.linear = linear;
            this.angular = angular;
        }

        public ChassisState() {
        }

        @Override
        public String toString() {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(linear) + ", " + fmt.format(angular);
        }
    }

    // Can refer to velocity, acceleration, torque, voltage, etc., depending on context.
    public static class WheelState {
        public double left;
        public double right;

        public WheelState(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public WheelState() {
        }

        public double get(boolean get_left) {
            return get_left ? left : right;
        }

        public void set(boolean set_left, double val) {
            if (set_left) {
                left = val;
            } else {
                right = val;
            }
        }

        @Override
        public String toString() {
            DecimalFormat fmt = new DecimalFormat("#0.000");
            return fmt.format(left) + ", " + fmt.format(right);
        }
    }

    // Full state dynamics of the drivetrain.
    // TODO maybe make these all optional fields and have a single solveDynamics() method that fills in the blanks?
    public static class DriveDynamics implements CSVWritable {
        public double curvature = 0.0;  // m^-1
        public double dcurvature = 0.0;  // m^-1/m
        public ChassisState chassis_velocity = new ChassisState();  // m/s
        public ChassisState chassis_acceleration = new ChassisState();  // m/s^2
        public WheelState wheel_velocity = new WheelState();  // rad/s
        public WheelState wheel_acceleration = new WheelState();  // rad/s^2
        public WheelState voltage = new WheelState();  // V
        public WheelState wheel_torque = new WheelState();  // N m

        @Override
        public String toCSV() {
            return curvature + "," + dcurvature + "," + chassis_velocity + ", " + chassis_acceleration + ", " + wheel_velocity + ", " + wheel_acceleration
                    + ", " + voltage + ", " + wheel_torque;
        }
    }
}
