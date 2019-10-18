package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.autonomous.controllers.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.math.Line;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Point;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class PurePursuitPath {
    public static double TRACK_SPEED = 0.5;
    private MecanumHardware robot;
    List<Waypoint> waypoints;

    // currPoint in 0..n-2 means we're on the path from waypoints[currPoint] to
    // waypoints[currPoint + 1]. currPoint = n-1 means we're done.
    int currPoint;

    public PurePursuitPath(MecanumHardware robot) {
        this(robot, new LinkedList<>());
    }

    public PurePursuitPath(MecanumHardware robot, Waypoint... points) {
        this(robot, Arrays.asList(points));
    }

    public PurePursuitPath(MecanumHardware robot, List<Waypoint> waypoints) {
        this.waypoints = waypoints;
        this.currPoint = 0;
        this.robot = robot;

        if (!(waypoints.get(waypoints.size() - 1) instanceof StopWaypoint)) {
            throw new IllegalArgumentException("Final Pure Pursuit waypoint must be a StopWaypoint!");
        }
    }

    public void update() {
        Pose robotPosition = robot.pose();

        System.out.println("Robot is at position " + robot.pose().toString());
        // Note - our currPoint will only be the last point in the list once we're done moving
        // the robot

        // Check whether we should advance to the next piece of the curve
        boolean jumpToNextSegment;
        do {
            jumpToNextSegment = false;
            Waypoint target = waypoints.get(currPoint + 1);
            if (target instanceof StopWaypoint) {
                if (robotPosition.distance(target) < ((StopWaypoint) target).allowedPositionError) {
                    jumpToNextSegment = true;
                }
            } else {
                if (robotPosition.distance(target) < target.followDistance) {
                    jumpToNextSegment = true;
                }
            }
            if (jumpToNextSegment) {
                currPoint++;
                waypoints.get(currPoint).runAction(robot);
            }
        } while (jumpToNextSegment && currPoint < waypoints.size() - 1);
        if (finished()) {return;}
        System.out.println("Currently on segment " + currPoint);

        Waypoint target = waypoints.get(currPoint + 1);
        // If we're making a stop and in the stop portion of the move
        if (target instanceof StopWaypoint && robotPosition.distance(target) < target.followDistance) {
            robot.setPowers(MecanumPurePursuitController.goToPosition(
                    robotPosition, target, TRACK_SPEED, false));
            System.out.println("Locking onto point " + target.toString());
        } else {
            trackToLine(
                    robotPosition,
                    waypoints.get(currPoint),
                    waypoints.get(currPoint + 1));
        }
    }

    /**
     * @param robotPosition a pose representing the current position and orientation of the robot
     * @param start the starting point of the line segment we're following. Can be any subclass of
     *             waypoint.
     * @param mid the end point of the line segment we're following. Must be a normal waypoint or
     *            a heading controlled waypoint. Passing a normal waypoint will cause the robot to
     *            turn itself in the direction of travel, while passing a heading controlled
     *            waypoint will cause the robot's heading to lock to the desired direction.
     */
    private void trackToLine(Pose robotPosition, Waypoint start, Waypoint mid) {
        Line currSegment = new Line(start, mid);
        Point center = currSegment.nearestLinePoint(robotPosition);

        Point intersection = MathUtil.lineSegmentCircleIntersection(
                start, mid, center, mid.followDistance
        );

        // If our line intersects at all
        // We clone the midpoint to preserve metadata, if it exists
        Waypoint target = mid.clone();
        target.x = intersection.x;
        target.y = intersection.y;
        robot.setPowers(MecanumPurePursuitController.goToPosition(robotPosition,
                target, TRACK_SPEED, true));
    }

    public Canvas draw(Canvas t) {
        double[] xPoints = new double[waypoints.size()];
        double[] yPoints = new double[waypoints.size()];

        for (int i = 0; i < waypoints.size(); i++) {
            xPoints[i] = waypoints.get(i).x;
            yPoints[i] = waypoints.get(i).y;
        }
        return t.setStroke("red").setStrokeWidth(1).strokePolyline(xPoints, yPoints);
    }

    public boolean finished() {
        return currPoint >= waypoints.size() - 1;
    }
}
