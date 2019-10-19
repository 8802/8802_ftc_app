package org.firstinspires.ftc.teamcode.common;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class StallPreventionQueueTest {

    @Test
    void testStallPreventionQueue() {
        StallPreventionQueue q = new StallPreventionQueue();
        q.add(Double.MAX_VALUE, 0);
        q.add(Double.MAX_VALUE, 100);

        // We haven't seen a full 500 millis worth of values yet
        assertFalse(q.aboveThresholdForMillis(100, 500, 200));

        q.add(Double.MAX_VALUE, 200);
        q.add(Double.MAX_VALUE, 300);
        q.add(Double.MAX_VALUE, 400);
        q.add(Double.MAX_VALUE, 500);
        q.add(Double.MAX_VALUE, 600);
        assertTrue(q.aboveThresholdForMillis(100, 500, 600));

        q.add(-0.01, 700);
        q.add(Double.MAX_VALUE, 800);
        assertFalse(q.aboveThresholdForMillis(100, 500, 200));

        // Assert no errors when we do things without the timestamp argument
        q.add(4);
        q.aboveThresholdForMillis(100, 500);

    }
}