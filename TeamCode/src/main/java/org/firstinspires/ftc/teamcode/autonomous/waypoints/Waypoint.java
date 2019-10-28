package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import org.firstinspires.ftc.teamcode.common.math.Point;

import java.util.Arrays;
import java.util.LinkedList;

public class Waypoint extends Point implements Cloneable {
    public Subroutines.Subroutine action;
    public double followDistance;

    public Waypoint(Point p, double followDistance) {
        this(p.x, p.y, followDistance, null);
    }
    public Waypoint(Point p, double followDistance, Subroutines.Subroutine action) {
        this(p.x, p.y, followDistance, action);
    }
    public Waypoint(double x, double y, double followDistance) {
        this(x, y, followDistance, null);
    }

    // Undeclared variables are null
    public Waypoint(double x, double y, double followDistance, Subroutines.Subroutine action) {
        super(x, y);
        this.action = action;
        this.followDistance = followDistance;
    }

    @Override
    public Waypoint clone() {
        return new Waypoint(x, y, followDistance, action);
    }

    public static LinkedList<Waypoint> collate(Waypoint... waypoints) {
        return new LinkedList<>(Arrays.asList(waypoints));
    }
}

