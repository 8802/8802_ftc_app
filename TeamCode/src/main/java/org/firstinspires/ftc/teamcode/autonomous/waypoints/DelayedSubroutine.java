package org.firstinspires.ftc.teamcode.autonomous.waypoints;

public class DelayedSubroutine {
    public long systemActionTime;
    public Subroutines.OnceOffSubroutine action;
    public String tag;

    public DelayedSubroutine(long timeFromNow, Subroutines.OnceOffSubroutine action) {
        this(timeFromNow, action, System.currentTimeMillis(), null);
    }

    public DelayedSubroutine(long timeFromNow, Subroutines.OnceOffSubroutine action, String tag) {
        this(timeFromNow, action, System.currentTimeMillis(), tag);
    }

    public DelayedSubroutine(long timeFromNow, Subroutines.OnceOffSubroutine action, long currentTime) {
        this(timeFromNow, action, currentTime, null);
    }

    public DelayedSubroutine(long timeFromNow, Subroutines.OnceOffSubroutine action, long currentTime, String tag) {
        this.systemActionTime = timeFromNow + currentTime;
        this.action = action;
        this.tag = tag;
    }
}