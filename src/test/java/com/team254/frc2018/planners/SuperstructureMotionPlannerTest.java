package com.team254.frc2018.planners;

import com.team254.frc2018.states.SuperstructureConstants;
import com.team254.frc2018.states.SuperstructureState;
import com.team254.util.test.ControlledActuatorLinearSim;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;


public class SuperstructureMotionPlannerTest {
    static final double ELEVATOR_MAX_HEIGHT = SuperstructureConstants.kElevatorMaxHeight;
    static final double ELEVATOR_MIN_HEIGHT = SuperstructureConstants.kElevatorMinHeight;
    static final double PIVOT_MAX_ANGLE = SuperstructureConstants.kWristMaxAngle;
    static final double PIVOT_MIN_ANGLE = SuperstructureConstants.kWristMinAngle;
    static final double ELEVATOR_VELOCITY = 70.0;
    static final double PIVOTY_VELOCITY = 180.0;
    static final double DELTA_T = 0.005;
    private static final double EPSILON = 1e-8;

    SuperstructureMotionPlanner planner = new SuperstructureMotionPlanner();
    SuperstructureState desiredState = new SuperstructureState();
    SuperstructureState simulatedState = new SuperstructureState();
    ControlledActuatorLinearSim elevatorSim = new ControlledActuatorLinearSim(ELEVATOR_MIN_HEIGHT,
            ELEVATOR_MAX_HEIGHT, ELEVATOR_VELOCITY);
    ControlledActuatorLinearSim pivotSim = new ControlledActuatorLinearSim(PIVOT_MIN_ANGLE, PIVOT_MAX_ANGLE,
            PIVOTY_VELOCITY);

    @Test
    public void testPlannerBootsToHome() {
        planner.setDesiredState(desiredState, simulatedState);
        SuperstructureState commandedState = planner.update(new SuperstructureState());
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must not go lower than low limit");
    }

    @Test
    public void testElevatorBottomLimit() {
        desiredState.height = -10;
        desiredState.angle = 0;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 0;
        planner.reset(simulatedState);
        planner.setDesiredState(desiredState, simulatedState);
        SuperstructureState commandedState = planner.update(simulatedState);
        assertEquals(PIVOT_MIN_ANGLE, commandedState.angle, EPSILON, "angle must be homed");
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must be homed");
    }

    @Test
    public void testCanMovePivotAtZeroHeight() {
        // 0 to 180
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 180;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 0;
        planner.setDesiredState(desiredState, simulatedState);
        SuperstructureState commandedState = planner.update(simulatedState);
        assertEquals(180, commandedState.angle, EPSILON, "angle must be correct");
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must be correct");

        // 180 to 0
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 0;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 180;
        planner.setDesiredState(desiredState, simulatedState);
        commandedState = planner.update(simulatedState);
        assertEquals(0, commandedState.angle, EPSILON, "angle must be correct");
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must be correct");

        // 0 to 90
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 90;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 0;
        planner.setDesiredState(desiredState, simulatedState);
        commandedState = planner.update(simulatedState);
        assertEquals(90, commandedState.angle, EPSILON, "angle must be correct");
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must be correct");
    }

    @Test
    public void testPivotSims() {
        SuperstructureState currentState = new SuperstructureState();
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 180;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 0;
        pivotSim.reset(simulatedState.angle);
        planner.reset(currentState);
        planner.setDesiredState(desiredState, simulatedState);

        for (double ts = 0; ts < 3.0; ts += DELTA_T) {
            SuperstructureState command = planner.update(currentState);
            pivotSim.setCommandedPosition(command.angle);
            double pivotAngle = pivotSim.update(DELTA_T);
            double raw = (ts + DELTA_T) * PIVOT_MAX_ANGLE;
            double expected = raw > 180 ? 180 : raw;
            assertEquals(expected, pivotAngle, EPSILON, "angle must be correct");
            currentState.angle = pivotAngle;
        }
    }

    @Test
    public void testWristDoesntGoTooLow() {
        SuperstructureState currentState = new SuperstructureState();
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 190;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 180;
        planner.setDesiredState(desiredState, simulatedState);

        // Sweep through height to make sure it doesnt go too low. Also makes it easy to modify
        // This test if we ever need to go lower than 180 at a tall height
        for (double height = ELEVATOR_MIN_HEIGHT; height <= ELEVATOR_MAX_HEIGHT; height = height + 20) {
            desiredState.angle = 190;
            desiredState.height = height;
            simulatedState.height = height;
            planner.setDesiredState(desiredState, simulatedState);
            SuperstructureState command = planner.update(currentState);
            assertEquals(PIVOT_MAX_ANGLE, command.angle, EPSILON, "wrist must not go past max limit");
        }
    }
/*
    @Test
    public void testWristStowsForBigMovements() {
        // Start with intake on ground
        simulatedState = new SuperstructureState();
        simulatedState.angle = PIVOT_MAX_ANGLE;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        pivotSim.reset(simulatedState.angle);
        elevatorSim.reset(simulatedState.height);

        // desire to move to top, wrist out straight
        desiredState.height = ELEVATOR_MAX_HEIGHT;
        desiredState.angle = PIVOT_MIN_ANGLE;

        planner.setDesiredState(desiredState, simulatedState);

        // Test that wrist is never less than vertical when elevator is "moving"
        for (double ts = 0; ts < 5; ts += DELTA_T) {
            SuperstructureState command = planner.update(simulatedState);
            pivotSim.setCommandedPosition(command.angle);
            simulatedState.angle = pivotSim.update(DELTA_T);
            elevatorSim.setCommandedPosition(command.height);
            simulatedState.height = elevatorSim.update(DELTA_T);

            boolean elevatorMoving = (simulatedState.height > ELEVATOR_MIN_HEIGHT + 1) &&
                    (simulatedState.height < ELEVATOR_MAX_HEIGHT - 1);
            boolean wristSafe = simulatedState.angle > 89;
            if (elevatorMoving) {
                if (!wristSafe) {
                    System.out.println("Wrist illegal: " + simulatedState + " " + command);
                }
                assertTrue(wristSafe, "wrist must be safe while moving");
            }
        }
    }

    @Test
    public void testCantCommandToIllegalCrossBarZone() {
        // Start with intake on ground
        simulatedState = new SuperstructureState();
        simulatedState.angle = PIVOT_MAX_ANGLE;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        planner.reset(simulatedState);
        pivotSim.reset(simulatedState.angle);
        elevatorSim.reset(simulatedState.height);

        // Move to an illegal zone
        desiredState.height = (SuperstructureConstants.kIllegalCrossbarStowMaxHeight + SuperstructureConstants
                .kIllegalCrossbarStowMinHeight) / 2.0;
        desiredState.angle = SuperstructureConstants.kIllegalCrossbarStowMinAngle - 5;

        boolean canMove = planner.setDesiredState(desiredState, simulatedState);
        assertFalse(canMove, "elevator shouldnt move to illegal position");

        // Test that wrist is never outside vertical when elevator is "moving"
        for (double ts = 0; ts < 5; ts += DELTA_T) {
            SuperstructureState command = planner.update(simulatedState);
            pivotSim.setCommandedPosition(command.angle);
            simulatedState.angle = pivotSim.update(DELTA_T);
            elevatorSim.setCommandedPosition(command.height);
            simulatedState.height = elevatorSim.update(DELTA_T);
            assertEquals(PIVOT_MAX_ANGLE, command.angle, EPSILON, "wrist must not move when commanded to illegal " +
                    "state");
            assertEquals(ELEVATOR_MIN_HEIGHT, command.height, EPSILON, "elevator must not move when commanded to " +
                    "illegal state");
        }
    }

    @Test
    public void testCantDunkOverTheBackTooFar() {
        // Start with intake on ground
        simulatedState = new SuperstructureState();
        simulatedState.angle = PIVOT_MAX_ANGLE;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        planner.reset(simulatedState);
        pivotSim.reset(simulatedState.angle);
        elevatorSim.reset(simulatedState.height);

        // Move to a sorta illegal but fixable dunking zone
        desiredState.height = SuperstructureConstants.kClearFirstStageMinHeight + 8;
        desiredState.angle = SuperstructureConstants.kClearFirstStageMinWristAngle - 15;

        boolean canMove = planner.setDesiredState(desiredState, simulatedState);
        assertTrue(canMove, "elevator planner should fix slightly illegal dunking zone");

        // Test that wrist is never outside vertical when elevator is "moving"
        for (double ts = 0; ts < 10; ts += DELTA_T) {
            SuperstructureState command = planner.update(simulatedState);
            pivotSim.setCommandedPosition(command.angle);
            simulatedState.angle = pivotSim.update(DELTA_T);
            elevatorSim.setCommandedPosition(command.height);
            simulatedState.height = elevatorSim.update(DELTA_T);

            assertFalse(simulatedState.inIllegalZone(), "went into an illegal zone");
        }
        assertEquals(SuperstructureConstants.kClearFirstStageMinWristAngle, simulatedState.angle, EPSILON, "wrist " +
                "must end in dunking position");
        assertEquals(SuperstructureConstants.kClearFirstStageMinHeight + 8, simulatedState.height, EPSILON, "elevator" +
                " must end in dunking position");
    }

    @Test
    public void testIsFinished() {
        // 0 to 180
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 180;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 0;
        planner.setDesiredState(desiredState, simulatedState);
        SuperstructureState commandedState = planner.update(simulatedState);
        assertEquals(180, commandedState.angle, EPSILON, "angle must be correct");
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must be correct");

        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 180;
        assertTrue(planner.isFinished(simulatedState));
    }

    @Test
    public void testChaosMonkeyControllingElevator() {
        // Start stowed
        simulatedState = new SuperstructureState();
        simulatedState.angle = PIVOT_MAX_ANGLE;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        planner.reset(simulatedState);
        pivotSim.reset(simulatedState.angle);
        elevatorSim.reset(simulatedState.height);
        SuperstructureState lastCommand = simulatedState;
        for (int i = 0; i < 50000; ++i) {
            double newHeight = Math.random() * (ELEVATOR_MAX_HEIGHT + 15);
            double newAngle = Math.random() * (PIVOT_MAX_ANGLE + 20);
            boolean jawClosed = Math.random() < .5;
            SuperstructureState commandedState = new SuperstructureState(newHeight, newAngle, jawClosed);
            boolean canMove = planner.setDesiredState(commandedState, simulatedState);
            if (canMove) {
                simulatedState.jawClamped = jawClosed;
            }

            for (double ts = 0; ts < 7; ts += DELTA_T) {
                SuperstructureState command = planner.update(simulatedState);
                pivotSim.setCommandedPosition(command.angle);
                simulatedState.angle = pivotSim.update(DELTA_T);
                elevatorSim.setCommandedPosition(command.height);
                simulatedState.height = elevatorSim.update(DELTA_T);

                if (simulatedState.inIllegalZone(true)) {
                    System.out.println("Discovered bad command: " + canMove + " : " + lastCommand + " to: " +
                            commandedState);
                    System.out.println("Bad simulatedState: " + simulatedState);
                    assertFalse(true, "went into an illegal zone");
                }

                if (simulatedState.inIllegalJawZone()) {
                    System.out.println("Discovered bad command: " + canMove + " : " + lastCommand + " to: " +
                            commandedState);
                    System.out.println("Bad simulatedState: " + simulatedState);
                    assertFalse(true, "went into an illegal jaw zone");
                }

            }
            lastCommand = commandedState;
        }
    }
*/

}
