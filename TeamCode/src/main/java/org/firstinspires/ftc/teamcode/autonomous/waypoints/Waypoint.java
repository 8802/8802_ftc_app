package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import org.firstinspires.ftc.teamcode.common.math.Point;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;

public class Waypoint extends Point implements Cloneable {
    Subroutine action;

    public Waypoint(Point p) {
        this(p.x, p.y, null);
    }
    public Waypoint(Point p, Subroutine action) {
        this(p.x, p.y, action);
    }
    public Waypoint(double x, double y) {
        this(x, y, null);
    }

    // Undeclared variables are null
    public Waypoint(double x, double y, Subroutine action) {
        super(x, y);
        this.action = action;
    }

    // Runs the action IF IT EXISTS
    public void runAction(MecanumHardware robot) {
        if (action != null) {
            action.run(robot);
        }
    }

    @Override
    public Waypoint clone() {
        return new Waypoint(x, y, action);
    }
}

