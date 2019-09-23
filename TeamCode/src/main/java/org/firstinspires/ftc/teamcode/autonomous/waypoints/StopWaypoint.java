package org.firstinspires.ftc.teamcode.autonomous.waypoints;

public class StopWaypoint extends HeadingControlledWaypoint {
    public double allowedPositionError;
    public double allowedHeadingError;

    public StopWaypoint(double x, double y, double followDistance, double targetHeading, double allowedPositionError) {
        this(x, y, targetHeading, followDistance, allowedPositionError, null);
    }

    public StopWaypoint(double x, double y, double followDistance, double targetHeading, double allowedPositionError, Subroutine action) {
        super(x, y, followDistance, targetHeading, action);
        this.allowedPositionError = allowedPositionError;
    }

    @Override
    public StopWaypoint clone() {
        return new StopWaypoint(x, y, followDistance, targetHeading, allowedPositionError, action);
    }
}
