package org.firstinspires.ftc.teamcode.common;

import java.util.LinkedList;

public class StallPreventionQueue {
    class TimeStampedValue {
        public double value;
        public long timestamp;

        public TimeStampedValue(double value, long timestamp) {
            this.value = value;
            this.timestamp = timestamp;
        }
    }

    public static double MAX_LENGTH = 1000;
    LinkedList<TimeStampedValue> queue;

    public StallPreventionQueue() {
        queue = new LinkedList<>();
    }

    public void add(double value) {
        this.add(value, System.currentTimeMillis());
    }

    public void add(double value, long timestamp) {
        queue.addFirst(new TimeStampedValue(value, timestamp));
        while (queue.size() > MAX_LENGTH) {
            queue.removeLast();
        }
    }

    public boolean aboveThresholdForMillis(double threshold, long millis) {
        return this.aboveThresholdForMillis(threshold, millis, System.currentTimeMillis());
    }

    public boolean aboveThresholdForMillis(double threshold, long millis, long timestamp) {
        long minTime = timestamp - millis;
        for (TimeStampedValue tick : queue) {
            // If we get here, we've iterated through enough of the list and haven't found any
            // disqualifying samples. We're good to return true.
            if (tick.timestamp < minTime) {
                return true;
            }

            // A single too low sample disqualifies us
            if (tick.value < threshold) {
                return false;
            }
        }
        // If we don't have a long enough data history to be sure, return false
        return false;
    }
}
