package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;

import java.util.LinkedList;

@Config
public class IntakeCurrentQueue {
    class TimeStampedValue {
        public IntakeCurrent value;
        public long timestamp;

        public TimeStampedValue(IntakeCurrent value, long timestamp) {
            this.value = value;
            this.timestamp = timestamp;
        }
    }

    public static double MAX_LENGTH = 1000;

    public static double STALL_DETECT_MAMPS = 8000;
    public static double STALL_DETECT_MS = 200;
    // How offset two spikes can be to still qualify as a block intake
    public static double BLOCK_INTAKE_SPIKE_OFFSET = 100;
    public static double BLOCK_INTAKE_DETECT_MAMPS = 3200;

    LinkedList<TimeStampedValue> queue;

    public IntakeCurrentQueue() {
        queue = new LinkedList<>();
    }

    public void add(IntakeCurrent value) {
        this.add(value, System.currentTimeMillis());
    }

    public void add(IntakeCurrent value, long timestamp) {
        queue.addFirst(new TimeStampedValue(value, timestamp));
        while (queue.size() > MAX_LENGTH) {
            queue.removeLast();
        }
    }

    public boolean stalled() {
        long minTime = queue.peekFirst().timestamp - (long) STALL_DETECT_MS;
        for (TimeStampedValue tick : queue) {
            // If we get here, we've iterated through enough of the list and haven't found any
            // disqualifying samples. We're good to return true.
            if (tick.timestamp < minTime) {
                return true;
            }

            // A single too low sample disqualifies us
            if (
                    tick.value.leftMAmps < STALL_DETECT_MAMPS &&
                    tick.value.rightMAmps < STALL_DETECT_MAMPS) {
                return false;
            }
        }
        return false; // For when queue is empty
    }

    public boolean hasBlock() {
        return queue.peekFirst().value.leftMAmps > BLOCK_INTAKE_DETECT_MAMPS && queue.peekFirst().value.rightMAmps > BLOCK_INTAKE_DETECT_MAMPS;
    }
}
