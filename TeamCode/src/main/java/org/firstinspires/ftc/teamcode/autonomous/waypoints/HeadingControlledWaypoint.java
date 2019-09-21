package org.firstinspires.ftc.teamcode.autonomous.waypoints;

public class HeadingControlledWaypoint extends Waypoint {
    public double targetHeading;

    public HeadingControlledWaypoint(double x, double y, double targetHeading) {
        this(x, y, targetHeading, null);
    }

    public HeadingControlledWaypoint(double x, double y, double targetHeading, Subroutine action) {
        super(x, y, action);
        this.targetHeading = targetHeading;
    }

    @Override
    public HeadingControlledWaypoint clone() {
        return new HeadingControlledWaypoint(x, y, targetHeading, action);
    }
}
