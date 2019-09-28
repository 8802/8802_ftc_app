package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class PoseTest {

    @Test
    void add() {
        Pose p1 = new Pose(5, 3, -2);
        // Heading is large and negative to ensure wrapping does not occur
        Pose p2 = new Pose(5, 3, -3000);
        Pose p3 = new Pose(5, 2, -2);

        // Unequal poses should be unequal regardless of which param is different
        assertNotEquals(p1, p2);
        assertNotEquals(p1, p3);

        // Addition should work and be commutative
        Pose sum = p1.add(p2);
        Pose sum2 = p2.add(p1);

        assertEquals(sum, sum2);
        assertEquals(sum, new Pose(10, 6, -3002));
    }

    @Test
    void multiply() {
        Pose p1 = new Pose(5, 3, -2);
        Pose p2 = new Pose(1, -3, -0.5);
        assertEquals(new Pose(5, -9, 1), p1.multiply(p2));

        // Assert we didn't change p1 and p2
        assertEquals(new Pose(5, 3, -2), p1);
        assertEquals(new Pose(1, -3, -0.5), p2);
    }

    @Test
    void scale() {
        Pose p1 = new Pose(5, 3, -2);
        assertEquals(new Pose(2.5, 1.5, -1), p1.scale(0.5));
        // Assert we didn't change p1
        assertEquals(new Pose(5, 3, -2), p1);
    }

    @Test
    void applyFriction() {
        Pose p1 = new Pose(50, -25, 6);
        p1.applyFriction(new Pose(5, 5, 10));
        assertEquals(new Pose(45, -20, 0), p1);
    }

    @Test
    void clampAbs() {
        Pose p1 = new Pose(50, -25, -2);
        p1.clampAbs(new Pose(5, 5, 5));
        assertEquals(new Pose(5, -5, -2), p1);
    }
}