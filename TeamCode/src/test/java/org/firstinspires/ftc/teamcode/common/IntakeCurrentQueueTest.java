package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.IntakeCurrent;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.IntakeCurrentQueue;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class IntakeCurrentQueueTest {

    @Test
    void testStallPrevention() {
        IntakeCurrent stallCurrents = new IntakeCurrent(12000, 12000);

        IntakeCurrentQueue q = new IntakeCurrentQueue();
        q.add(stallCurrents, 0);
        q.add(stallCurrents, 100);

        // We haven't seen a full 500 millis worth of values yet
        assertFalse(q.stalled());

        q.add(new IntakeCurrent(12000, 0), 200);
        q.add(new IntakeCurrent(0, 12000), 300);
        q.add(stallCurrents, 400);
        q.add(stallCurrents, 500);
        q.add(stallCurrents, 600);
        assertTrue(q.stalled());

        q.add(new IntakeCurrent(1500, 2000), 700);
        q.add(stallCurrents, 800);
        assertFalse(q.stalled());

        // Assert no errors when we do things without the timestamp argument
        q.add(new IntakeCurrent(0, 0));
    }
}