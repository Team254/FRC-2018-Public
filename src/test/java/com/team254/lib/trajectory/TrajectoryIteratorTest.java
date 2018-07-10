package com.team254.lib.trajectory;

import com.team254.lib.geometry.Translation2d;
import com.team254.lib.util.Util;
import org.junit.jupiter.api.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

public class TrajectoryIteratorTest {
    public static final double kTestEpsilon = Util.kEpsilon;

    public static final List<Translation2d> kWaypoints = Arrays.asList(
            new Translation2d(0.0, 0.0),
            new Translation2d(24.0, 0.0),
            new Translation2d(36.0, 12.0),
            new Translation2d(60.0, 12.0));

    @Test
    public void test() {
        Trajectory<Translation2d> traj = new Trajectory<>(kWaypoints);
        TrajectoryIterator<Translation2d> iterator = new TrajectoryIterator<>(traj.getIndexView());

        // Initial conditions.
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertEquals(kWaypoints.get(0), iterator.getState());
        assertFalse(iterator.isDone());

        // Advance forward.
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5), iterator.preview(0.5).state());
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.5), iterator.advance(0.5).state());
        assertEquals(0.5, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.5, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance backwards.
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25), iterator.preview(-0.25).state());
        assertEquals(kWaypoints.get(0).interpolate(kWaypoints.get(1), 0.25), iterator.advance(-0.25).state());
        assertEquals(0.25, iterator.getProgress(), kTestEpsilon);
        assertEquals(2.75, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());

        // Advance past end.
        assertEquals(kWaypoints.get(3), iterator.preview(5.0).state());
        assertEquals(kWaypoints.get(3), iterator.advance(5.0).state());
        assertEquals(3.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(0.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertTrue(iterator.isDone());

        // Advance past beginning.
        assertEquals(kWaypoints.get(0), iterator.preview(-5.0).state());
        assertEquals(kWaypoints.get(0), iterator.advance(-5.0).state());
        assertEquals(0.0, iterator.getProgress(), kTestEpsilon);
        assertEquals(3.0, iterator.getRemainingProgress(), kTestEpsilon);
        assertFalse(iterator.isDone());
    }

}
