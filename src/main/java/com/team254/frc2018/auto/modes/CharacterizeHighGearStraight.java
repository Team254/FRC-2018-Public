package com.team254.frc2018.auto.modes;

import com.team254.frc2018.auto.AutoModeBase;
import com.team254.frc2018.auto.AutoModeEndedException;
import com.team254.frc2018.auto.actions.CollectAccelerationData;
import com.team254.frc2018.auto.actions.CollectVelocityData;
import com.team254.frc2018.auto.actions.WaitAction;
import com.team254.lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeHighGearStraight extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {
        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();

        // runAction(new ShiftHighGearAction(false));
        // runAction(new WaitAction(10));

        runAction(new CollectVelocityData(velocityData, false, false, true));
        runAction(new WaitAction(10));
        runAction(new CollectAccelerationData(accelerationData, false, false, true));

        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);

        System.out.println("ks: " + constants.ks);
        System.out.println("kv: " + constants.kv);
        System.out.println("ka: " + constants.ka);
    }
}
