package com.team254.lib.physics;

import com.team254.lib.util.Util;

public class PhysicsTest {
    public static final double kTestEpsilon = Util.kEpsilon;
    /*

    @Test
    public void testDCMotorTransmission() {
        // 100 rpm per V, .2 N*m per V, 1.5V to overcome friction.
        DCMotorTransmission motor = new DCMotorTransmission(Units.rpm_to_rads_per_sec(100.0), .2, 1.5);

        assertEquals(Units.rpm_to_rads_per_sec(1200.0), motor.free_speed_at_voltage(12.0 + 1.5));
        assertEquals(Units.rpm_to_rads_per_sec(600.0), motor.free_speed_at_voltage(6.0 + 1.5));
        assertEquals(Units.rpm_to_rads_per_sec(0.0), motor.free_speed_at_voltage(1.4));
        assertEquals(Units.rpm_to_rads_per_sec(0.0), motor.free_speed_at_voltage(0.0));
        assertEquals(Units.rpm_to_rads_per_sec(0.0), motor.free_speed_at_voltage(-1.4));
        assertEquals(Units.rpm_to_rads_per_sec(-600.0), motor.free_speed_at_voltage(-6.0 - 1.5));
        assertEquals(Units.rpm_to_rads_per_sec(-1200.0), motor.free_speed_at_voltage(-12.0 - 1.5));

        assertEquals(.2 * 10.5, motor.getTorqueForVoltage(0.0, 12.0));
        assertEquals(.2 * 3.5, motor.getTorqueForVoltage(0.0, 5.0));
        assertEquals(-.2 * 10.5, motor.getTorqueForVoltage(0.0, -12.0));
        assertEquals(0.0, motor.getTorqueForVoltage(0.0, 0.0));
        assertEquals(0.0, motor.getTorqueForVoltage(0.0, 1.4));
        assertEquals(0.0, motor.getTorqueForVoltage(0.0, -1.4));

        assertEquals(0.0, motor.getTorqueForVoltage(Units.rpm_to_rads_per_sec(1200.0), 13.5));
        assertEquals(-.2 * 1.5, motor.getTorqueForVoltage(Units.rpm_to_rads_per_sec(1200.0), 12.0));
        assertEquals(.2 * 1.5, motor.getTorqueForVoltage(Units.rpm_to_rads_per_sec(1200.0), 15.0));

        assertEquals(0.0, motor.getVoltageForTorque(0.0, 0.0));
        assertEquals(13.5, motor.getVoltageForTorque(Units.rpm_to_rads_per_sec(1200.0), 0.0));
        assertEquals(-13.5, motor.getVoltageForTorque(Units.rpm_to_rads_per_sec(-1200.0), 0.0));

        double[] voltages = new double[]{0.0, 1.0, 3.34, 6.4, -2.0, 0.9, -5.6, 12.1, 1.499, 1.501};
        double[] speeds = new double[]{0.0, 0.1, -0.5, 130.1, 3000.0, -45.0, 666.666};

        for (double speed : speeds) {
            for (double voltage : voltages) {
                double torque = motor.getTorqueForVoltage(Units.rpm_to_rads_per_sec(speed), voltage);
                if (Math.abs(voltage) <= 1.5 && Util.epsilonEquals(speed, 0.0)) {
                    assertEquals(0.0, torque);
                } else {
                    assertEquals(voltage, motor.getVoltageForTorque(Units.rpm_to_rads_per_sec(speed), torque), 1e-9);
                }
            }
        }
    }

    @Test
    public void testDifferentialDrive() {
        DCMotorTransmission transmission = new DCMotorTransmission(Units.rpm_to_rads_per_sec(65.0), 0.35, 1.0);
        DifferentialDrive drive = new DifferentialDrive(70.0, 84.0, Units.inches_to_meters(2.0), Units
                .inches_to_meters(25.5) / 2.0, transmission, transmission);

        // Kinematics
        DifferentialDrive.ChassisState velocity = drive.solveForwardKinematics(new DifferentialDrive.WheelState(0.0,
                0.0));
        assertEquals(0.0, velocity.linear);
        assertEquals(0.0, velocity.angular);
        DifferentialDrive.WheelState wheels = drive.solveInverseKinematics(velocity);
        assertEquals(0.0, wheels.left);
        assertEquals(0.0, wheels.right);
        velocity = drive.solveForwardKinematics(new DifferentialDrive.WheelState(Units.rpm_to_rads_per_sec(65.0 *
                10.0), Units.rpm_to_rads_per_sec(65.0 * 10.0)));
        assertEquals(11.0, Units.meters_to_feet(velocity.linear), 1.0);
        assertEquals(0.0, velocity.angular);
        wheels = drive.solveInverseKinematics(velocity);
        assertEquals(Units.rpm_to_rads_per_sec(65.0 * 10.0), wheels.left);
        assertEquals(Units.rpm_to_rads_per_sec(65.0 * 10.0), wheels.right);
        velocity = drive.solveForwardKinematics(new DifferentialDrive.WheelState(Units.rpm_to_rads_per_sec(65.0 *
                -10.0), Units.rpm_to_rads_per_sec(65.0 * -10.0)));
        assertEquals(-11.0, Units.meters_to_feet(velocity.linear), 1.0);
        assertEquals(0.0, velocity.angular);
        wheels = drive.solveInverseKinematics(velocity);
        assertEquals(Units.rpm_to_rads_per_sec(-65.0 * 10.0), wheels.left);
        assertEquals(Units.rpm_to_rads_per_sec(-65.0 * 10.0), wheels.right);
        velocity = drive.solveForwardKinematics(new DifferentialDrive.WheelState(Units.rpm_to_rads_per_sec(-65.0 *
                10.0), Units.rpm_to_rads_per_sec(65.0 * 10.0)));
        assertEquals(0.0, Units.meters_to_feet(velocity.linear), 1.0);
        assertEquals(10.0, velocity.angular, 1.0);
        wheels = drive.solveInverseKinematics(velocity);
        assertEquals(Units.rpm_to_rads_per_sec(-65.0 * 10.0), wheels.left);
        assertEquals(Units.rpm_to_rads_per_sec(65.0 * 10.0), wheels.right);
        velocity = drive.solveForwardKinematics(new DifferentialDrive.WheelState(Units.rpm_to_rads_per_sec(65.0 *
                5.0), Units.rpm_to_rads_per_sec(-65.0 * 5.0)));
        assertEquals(0.0, Units.meters_to_feet(velocity.linear), 1.0);
        assertEquals(-5.0, velocity.angular, 1.0);
        wheels = drive.solveInverseKinematics(velocity);
        assertEquals(Units.rpm_to_rads_per_sec(65.0 * 5.0), wheels.left);
        assertEquals(Units.rpm_to_rads_per_sec(-65.0 * 5.0), wheels.right);

        // Forward dynamics.
        DifferentialDrive.DriveDynamics dynamics = drive.solveForwardDynamics(
                new DifferentialDrive.ChassisState(0.0, 0.0),
                new DifferentialDrive.WheelState(0.0, 0.0));
        assertEquals(0.0, dynamics.wheel_torque.left);
        assertEquals(0.0, dynamics.wheel_torque.right);
        assertEquals(0.0, dynamics.wheel_acceleration.left);
        assertEquals(0.0, dynamics.wheel_acceleration.right);
        assertEquals(0.0, dynamics.chassis_acceleration.linear);
        assertEquals(0.0, dynamics.chassis_acceleration.angular);
        dynamics = drive.solveForwardDynamics(
                new DifferentialDrive.ChassisState(0.0, 0.0),
                new DifferentialDrive.WheelState(12.0, 12.0));
        assertEquals(11.0 * .35, dynamics.wheel_torque.left);
        assertEquals(11.0 * .35, dynamics.wheel_torque.right);
        assertTrue(0.0 < dynamics.wheel_acceleration.left);
        assertTrue(0.0 < dynamics.wheel_acceleration.right);
        assertEquals(2.0, dynamics.chassis_acceleration.linear, 1.0);
        assertEquals(0.0, dynamics.chassis_acceleration.angular);
        dynamics = drive.solveForwardDynamics(
                new DifferentialDrive.ChassisState(0.0, 0.0),
                new DifferentialDrive.WheelState(-12.0, -12.0));
        assertEquals(-11.0 * .35, dynamics.wheel_torque.left);
        assertEquals(-11.0 * .35, dynamics.wheel_torque.right);
        assertTrue(0.0 > dynamics.wheel_acceleration.left);
        assertTrue(0.0 > dynamics.wheel_acceleration.right);
        assertTrue(0.0 > dynamics.chassis_acceleration.linear);
        assertEquals(0.0, dynamics.chassis_acceleration.angular);
        dynamics = drive.solveForwardDynamics(
                new DifferentialDrive.ChassisState(0.0, 0.0),
                new DifferentialDrive.WheelState(-12.0, 12.0));
        assertEquals(-11.0 * .35, dynamics.wheel_torque.left);
        assertEquals(11.0 * .35, dynamics.wheel_torque.right);
        assertTrue(0.0 > dynamics.wheel_acceleration.left);
        assertTrue(0.0 < dynamics.wheel_acceleration.right);
        assertEquals(0.0, dynamics.chassis_acceleration.linear);
        assertTrue(0.0 < dynamics.chassis_acceleration.angular);

        // Inverse dynamics.
        dynamics = drive.solveInverseDynamics(
                new DifferentialDrive.ChassisState(0.0, 0.0),
                new DifferentialDrive.ChassisState(0.0, 0.0));
        assertEquals(0.0, dynamics.wheel_torque.left);
        assertEquals(0.0, dynamics.wheel_torque.right);
        assertEquals(0.0, dynamics.voltage.left);
        assertEquals(0.0, dynamics.voltage.right);
        dynamics = drive.solveInverseDynamics(
                new DifferentialDrive.ChassisState(Units.feet_to_meters(10.0), 0.0),
                new DifferentialDrive.ChassisState(0.0, 0.0));
        assertEquals(0.0, dynamics.wheel_torque.left);
        assertEquals(0.0, dynamics.wheel_torque.right);
        assertEquals(9.5, dynamics.voltage.left, 1.0);
        assertEquals(9.5, dynamics.voltage.right, 1.0);
        dynamics = drive.solveInverseDynamics(
                new DifferentialDrive.ChassisState(Units.feet_to_meters(10.0), 0.0),
                new DifferentialDrive.ChassisState(Units.feet_to_meters(2.0), 0.0));
        assertEquals(1.0, dynamics.wheel_torque.left, 0.5);
        assertEquals(1.0, dynamics.wheel_torque.right, 0.5);
        assertEquals(13.0, dynamics.voltage.left, 1.0);
        assertEquals(13.0, dynamics.voltage.right, 1.0);
        dynamics = drive.solveInverseDynamics(
                new DifferentialDrive.ChassisState(Units.feet_to_meters(10.0), 0.0),
                new DifferentialDrive.ChassisState(Units.feet_to_meters(-2.0), 0.0));
        assertEquals(-1.0, dynamics.wheel_torque.left, 0.5);
        assertEquals(-1.0, dynamics.wheel_torque.right, 0.5);
        assertEquals(6.5, dynamics.voltage.left, 1.0);
        assertEquals(6.5, dynamics.voltage.right, 1.0);
        dynamics = drive.solveInverseDynamics(
                new DifferentialDrive.ChassisState(Units.feet_to_meters(10.0), Units.degrees_to_radians(45.0)),
                new DifferentialDrive.ChassisState(Units.feet_to_meters(2.0), Units.degrees_to_radians(9.0)));
        assertEquals(1.0, dynamics.wheel_torque.left, 0.5);
        assertEquals(1.0, dynamics.wheel_torque.right, 0.5);
        assertEquals(11.0, dynamics.voltage.left, 1.0);
        assertEquals(14.0, dynamics.voltage.right, 1.0);

        // Max speed.
        assertEquals(Units.feet_to_meters(13.0), drive.getMaxAbsVelocity(0.0, 12.0), 1.0);
        assertEquals(Units.feet_to_meters(6.0), drive.getMaxAbsVelocity(0.0, 6.0), 1.0);
        assertEquals(Units.feet_to_meters(3.0), drive.getMaxAbsVelocity(1.0 / drive.effective_wheelbase_radius(),
                6.0), 1.0);
        assertEquals(Units.feet_to_meters(3.0), drive.getMaxAbsVelocity(-1.0 / drive.effective_wheelbase_radius(),
                6.0), 1.0);
        assertEquals(Units.rpm_to_rads_per_sec(50.0), drive.getMaxAbsVelocity(Double.POSITIVE_INFINITY,
                6.0), 1.0);
        assertEquals(Units.rpm_to_rads_per_sec(-50.0), drive.getMaxAbsVelocity(Double.NEGATIVE_INFINITY,
                6.0), 1.0);

        // Max acceleration.
        DifferentialDrive.MinMax min_max_accel;
        min_max_accel = drive.getMinMaxAcceleration(new DifferentialDrive.ChassisState(0.0, 0.0), 0.0, 12.0);
        assertEquals(2.0, min_max_accel.max, 1.0);
        assertEquals(-2.0, min_max_accel.min, 1.0);
        min_max_accel = drive.getMinMaxAcceleration(new DifferentialDrive.ChassisState(0.0, 0.0), 0.0, 6.0);
        assertEquals(1.0, min_max_accel.max, 0.5);
        assertEquals(-1.0, min_max_accel.min, 0.5);
        min_max_accel = drive.getMinMaxAcceleration(new DifferentialDrive.ChassisState(Units.feet_to_meters(8.0),
                0.0), 0.0, 12.0);
        assertEquals(1.0, min_max_accel.max, 1.0);
        assertEquals(-4.0, min_max_accel.min, 1.0);
        min_max_accel = drive.getMinMaxAcceleration(new DifferentialDrive.ChassisState(0.0, 0.0), Double
                .POSITIVE_INFINITY, 6.0);
        assertEquals(1.0, min_max_accel.max, 0.5);
        assertEquals(-1.0, min_max_accel.min, 0.5);
    }
    */
}
