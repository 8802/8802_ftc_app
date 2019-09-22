package org.firstinspires.ftc.teamcode.autonomous.waypoints;

public class StopWaypoint extends HeadingControlledWaypoint {
    public double allowedError;

    public StopWaypoint(double x, double y, double followDistance, double targetHeading, double allowableError) {
        this(x, y, targetHeading, followDistance, allowableError, null);
    }

    public StopWaypoint(double x, double y, double followDistance, double targetHeading, double allowableError, Subroutine action) {
        super(x, y, followDistance, targetHeading, action);
        this.targetHeading = targetHeading;
    }

    @Override
    public StopWaypoint clone() {
        return new StopWaypoint(x, y, followDistance, targetHeading, allowedError, action);
    }
}
