package org.firstinspires.ftc.teamcode.autonomous.waypoints;

public class HeadingControlledWaypoint extends Waypoint {
    public double targetHeading;

    public HeadingControlledWaypoint(double x, double y, double followDistance, double targetHeading) {
        this(x, y, followDistance, targetHeading, null);
    }

    public HeadingControlledWaypoint(double x, double y, double followDistance, double targetHeading, Subroutine action) {
        super(x, y, followDistance, action);
        this.targetHeading = targetHeading;
    }

    @Override
    public HeadingControlledWaypoint clone() {
        return new HeadingControlledWaypoint(x, y, followDistance, targetHeading, action);
    }
}
