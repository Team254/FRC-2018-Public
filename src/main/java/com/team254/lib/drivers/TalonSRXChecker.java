package com.team254.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team254.frc2018.subsystems.Subsystem;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class TalonSRXChecker {
    public static class CheckerConfig {
        public double mCurrentFloor = 5;
        public double mRPMFloor = 2000;

        public double mCurrentEpsilon = 5.0;
        public double mRPMEpsilon = 500;
        public DoubleSupplier mRPMSupplier = null;

        public double mRunTimeSec = 4.0;
        public double mWaitTimeSec = 2.0;
        public double mRunOutputPercentage = 0.5;
    }

    public static class TalonSRXConfig {
        public String mName;
        public TalonSRX mTalon;

        public TalonSRXConfig(String name, TalonSRX talon) {
            mName = name;
            mTalon = talon;
        }
    }

    private static class StoredTalonSRXConfiguration {
        public ControlMode mMode;
        public double mSetValue;
    }

    public static boolean CheckTalons(Subsystem subsystem,
                                      ArrayList<TalonSRXConfig> talonsToCheck,
                                      CheckerConfig checkerConfig) {
        boolean failure = false;
        System.out.println("////////////////////////////////////////////////");
        System.out.println("Checking subsystem " + subsystem.getClass()
                + " for " + talonsToCheck.size() + " talons.");

        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();
        ArrayList<StoredTalonSRXConfiguration> storedConfigurations = new ArrayList<>();

        // Record previous configuration for all talons.
        for (TalonSRXConfig config : talonsToCheck) {
            LazyTalonSRX talon = LazyTalonSRX.class.cast(config.mTalon);

            StoredTalonSRXConfiguration configuration = new StoredTalonSRXConfiguration();
            configuration.mMode = talon.getControlMode();
            configuration.mSetValue = talon.getLastSet();

            storedConfigurations.add(configuration);

            // Now set to disabled.
            talon.set(ControlMode.PercentOutput, 0.0);
        }

        for (TalonSRXConfig config : talonsToCheck) {
            System.out.println("Checking: " + config.mName);

            config.mTalon.set(ControlMode.PercentOutput, checkerConfig.mRunOutputPercentage);
            Timer.delay(checkerConfig.mRunTimeSec);

            // Now poll the interesting information.
            double current = config.mTalon.getOutputCurrent();
            currents.add(current);
            System.out.print("Current: " + current);

            double rpm = Double.NaN;
            if (checkerConfig.mRPMSupplier != null) {
                rpm = checkerConfig.mRPMSupplier.getAsDouble();
                rpms.add(rpm);
                System.out.print(" RPM: " + rpm);
            }
            System.out.print('\n');

            config.mTalon.set(ControlMode.PercentOutput, 0.0);

            // And perform checks.
            if (current < checkerConfig.mCurrentFloor) {
                System.out.println(config.mName + " has failed current floor check vs " +
                        checkerConfig.mCurrentFloor + "!!");
                failure = true;
            }
            if (checkerConfig.mRPMSupplier != null) {
                if (rpm < checkerConfig.mRPMFloor) {
                    System.out.println(config.mName + " has failed rpm floor check vs " +
                            checkerConfig.mRPMFloor + "!!");
                    failure = true;
                }
            }

            Timer.delay(checkerConfig.mWaitTimeSec);
        }

        // Now run aggregate checks.

        if (currents.size() > 0) {
            Double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(currents, average, checkerConfig.mCurrentEpsilon)) {
                System.out.println("Currents varied!!!!!!!!!!!");
                failure = true;
            }
        }

        if (rpms.size() > 0) {
            Double average = rpms.stream().mapToDouble(val -> val).average().getAsDouble();

            if (!Util.allCloseTo(rpms, average, checkerConfig.mRPMEpsilon)) {
                System.out.println("RPMs varied!!!!!!!!");
                failure = true;
            }
        }

        // Restore Talon configurations
        for (int i = 0; i < talonsToCheck.size(); ++i) {
            talonsToCheck.get(i).mTalon.set(storedConfigurations.get(i).mMode,
                    storedConfigurations.get(i).mSetValue);
        }

        return !failure;
    }
}
