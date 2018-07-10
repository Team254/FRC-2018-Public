package com.team254.frc2018.statemachines;

public class TestIntakeStateMachine {
    /*
    IntakeStateMachine sm = new IntakeStateMachine();
    IntakeStateMachine.WantedAction action = IntakeStateMachine.WantedAction.IDLE;
    IntakeState simState = new IntakeState();

    private static final double EPSILON = 1e-8;

    @Test
    public void testBoot() {
        IntakeState commandedState = sm.update(0, action, simState);
        assertEquals(0, commandedState.leftMotor, EPSILON, "motor must default to 0");
        assertEquals(0, commandedState.rightMotor, EPSILON, "motor must default to 0");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);
    }


    @Test
    public void testGoesToIntaking() {
        action = IntakeStateMachine.WantedAction.INTAKE;
        IntakeState commandedState = sm.update(0, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);
    }

    @Test
    public void testGoesToIdle() {
        action = IntakeStateMachine.WantedAction.IDLE;
        IntakeState commandedState = sm.update(0, action, simState);
        assertEquals(0, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(0, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);
    }

    @Test
    public void testDoesClampClean() {
        double ts = 0;
        action = IntakeStateMachine.WantedAction.INTAKE;
        // iter 1
        IntakeState commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);
        assertFalse(sm.hasCubeClamped(), "doesnt have cube yet");

        // iter 2
        ts += 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);
        assertFalse(sm.hasCubeClamped(), "doesnt have cube yet");


        // iter 3
        ts += 0.005;
        simState.leftCubeSensorTriggered = true;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);
        assertFalse(sm.hasCubeClamped(), "doesnt have cube yet");

        // iter 4, jump forward in time
        simState.leftCubeSensorTriggered = true;
        ts += IntakeStateMachine.kActuationTime + 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        assertTrue(sm.hasCubeClamped(), "has cube");
    }

    @Test
    public void testCantOpenWhileInAFrame() {
        double ts = 0;
        action = IntakeStateMachine.WantedAction.INTAKE;
        // iter 1
        IntakeState commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);

        // iter 2
        ts += 0.005;
        simState.leftCubeSensorTriggered = true;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 3, jump forward in time
        simState.leftCubeSensorTriggered = true;
        ts += IntakeStateMachine.kActuationTime + 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 4 - try to place, but wrist is in a frame
        action = IntakeStateMachine.WantedAction.PLACE;
        simState.wristAngle = 0;
        ts += 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);
    }

    @Test
    public void testCanPlace() {
        double ts = 0;
        action = IntakeStateMachine.WantedAction.INTAKE;
        // iter 1
        IntakeState commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);

        // iter 2
        ts += 0.005;
        simState.leftCubeSensorTriggered = true;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 3, jump forward in time
        simState.leftCubeSensorTriggered = true;
        ts += IntakeStateMachine.kActuationTime + 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 4 - try to place,  wrist is NOT in a frame
        action = IntakeStateMachine.WantedAction.PLACE;
        simState.wristAngle = 175;
        ts += 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.OPEN);
    }

    @Test
    public void testCanShootForwards() {
        double ts = 0;
        action = IntakeStateMachine.WantedAction.INTAKE;
        // iter 1
        IntakeState commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);

        // iter 2
        ts += 0.005;
        simState.leftCubeSensorTriggered = true;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 3, jump forward in time
        simState.leftCubeSensorTriggered = true;
        ts += IntakeStateMachine.kActuationTime + 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 4 - try to place,  wrist is NOT in a frame
        action = IntakeStateMachine.WantedAction.SHOOT;
        simState.wristAngle = 175;
        ts += 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kShootSetpoint, commandedState.leftMotor, EPSILON, "motor must shoot");
        assertEquals(IntakeStateMachine.kShootSetpoint, commandedState.rightMotor, EPSILON, "motor must shoot");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);
    }

    @Test
    public void testCanShootBackwards() {
        double ts = 0;
        action = IntakeStateMachine.WantedAction.INTAKE;
        // iter 1
        IntakeState commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLOSED);

        // iter 2
        ts += 0.005;
        simState.leftCubeSensorTriggered = true;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.leftMotor, EPSILON, "motor must intake");
        assertEquals(IntakeStateMachine.kIntakeCubeSetpoint, commandedState.rightMotor, EPSILON, "motor must intake");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 3, jump forward in time
        simState.leftCubeSensorTriggered = true;
        ts += IntakeStateMachine.kActuationTime + 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.leftMotor, EPSILON, "motor must hold");
        assertEquals(IntakeStateMachine.kHoldSetpoint, commandedState.rightMotor, EPSILON, "motor must hold");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);

        // iter 4 - try to shoot,  wrist is in a frame
        action = IntakeStateMachine.WantedAction.SHOOT;
        simState.wristAngle = 0;
        ts += 0.005;
        commandedState = sm.update(ts, action, simState);
        assertEquals(IntakeStateMachine.kShootSetpoint, commandedState.leftMotor, EPSILON, "motor must shoot");
        assertEquals(IntakeStateMachine.kShootSetpoint, commandedState.rightMotor, EPSILON, "motor must shoot");
        assertEquals(commandedState.jawState, IntakeState.JawState.CLAMPED);
    }*/
}
