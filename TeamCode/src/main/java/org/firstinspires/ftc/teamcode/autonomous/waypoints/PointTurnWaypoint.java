package org.firstinspires.ftc.teamcode.autonomous.waypoints;

public class PointTurnWaypoint extends HeadingControlledWaypoint {
    public double allowedHeadingError;

    public PointTurnWaypoint(double x, double y, double followDistance, double targetHeading, double allowedHeadingError) {
        this(x, y, followDistance, targetHeading, allowedHeadingError, null);
    }

    public PointTurnWaypoint(double x, double y, double followDistance, double targetHeading, double allowedHeadingError, Subroutines.Subroutine action) {
        super(x, y, followDistance, targetHeading, action);
        this.allowedHeadingError = allowedHeadingError;
    }

    @Override
    public PointTurnWaypoint clone() {
        return new PointTurnWaypoint(x, y, followDistance, targetHeading, allowedHeadingError, action);
    }
}
