package org.firstinspires.ftc.teamcode.autonomous.waypoints;

public class StopWaypoint extends HeadingControlledWaypoint {
    public double allowedError;

    public StopWaypoint(double x, double y, double targetHeading, double allowableError) {
        this(x, y, targetHeading, allowableError, null);
    }

    public StopWaypoint(double x, double y, double targetHeading, double allowableError, Subroutine action) {
        super(x, y, targetHeading, action);
        this.targetHeading = targetHeading;
    }

    @Override
    public StopWaypoint clone() {
        return new StopWaypoint(x, y, targetHeading, allowedError, action);
    }
}
