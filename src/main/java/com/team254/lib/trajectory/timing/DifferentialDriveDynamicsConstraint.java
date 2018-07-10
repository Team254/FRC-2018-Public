package com.team254.lib.trajectory.timing;

import com.team254.lib.geometry.ICurvature;
import com.team254.lib.geometry.IPose2d;
import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.util.Units;

public class DifferentialDriveDynamicsConstraint<S extends IPose2d<S> & ICurvature<S>> implements TimingConstraint<S> {

    protected final DifferentialDrive drive_;
    protected final double abs_voltage_limit_;

    public DifferentialDriveDynamicsConstraint(final DifferentialDrive drive, double abs_voltage_limit) {
        drive_ = drive;
        abs_voltage_limit_ = abs_voltage_limit;
    }

    @Override
    public double getMaxVelocity(S state) {
        return Units.meters_to_inches(drive_.getMaxAbsVelocity(
                Units.meters_to_inches(state.getCurvature()),  // Curvature is in inverse inches, so meters_to_inches is correct.
                /*Units.meters_to_inches(Units.meters_to_inches(state.getDCurvatureDs())),  // DCurvature is in inverse inches^2.*/
                abs_voltage_limit_));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(S state,
                                                    double velocity) {
        // TODO figure out a units convention for generic states.  Traditionally we use inches...
        // NOTE: units cancel on angular velocity.
        DifferentialDrive.MinMax min_max = drive_.getMinMaxAcceleration(new DifferentialDrive.ChassisState(
                        Units.inches_to_meters(velocity), state.getCurvature() * velocity),
                Units.meters_to_inches(state.getCurvature()),  // Curvature is in inverse inches, so meters_to_inches is correct.
                /*Units.meters_to_inches(Units.meters_to_inches(state.getDCurvatureDs())),  // DCurvature is in inverse inches^2.*/
                abs_voltage_limit_);
        return new MinMaxAcceleration(Units.meters_to_inches(min_max.min), Units.meters_to_inches(min_max.max));
    }
}
