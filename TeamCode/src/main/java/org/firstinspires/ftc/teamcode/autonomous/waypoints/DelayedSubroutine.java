package org.firstinspires.ftc.teamcode.autonomous.waypoints;

public class DelayedSubroutine {
    public long systemActionTime;
    public Subroutines.OnceOffSubroutine action;

    public DelayedSubroutine(long timeFromNow, Subroutines.OnceOffSubroutine action) {
        this(timeFromNow, action, System.currentTimeMillis());
    }

    public DelayedSubroutine(long timeFromNow, Subroutines.OnceOffSubroutine action, long currentTime) {
        this.systemActionTime = timeFromNow + currentTime;
        this.action = action;
    }
}