package org.firstinspires.ftc.teamcode.autonomous.controllers;

import org.firstinspires.ftc.teamcode.common.math.Line;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Point;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.teleop.MecanumRobotCentric;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class PurePursuitPath {
    public static double LOOK_AHEAD_DISTANCE = 16;

    private MecanumHardware robot;
    List<Point> waypoints;

    // currPoint in 0..n-2 means we're on the path from waypoints[currPoint] to
    // waypoints[currPoint + 1]. currPoint = n-1 means we're zeroing in on the last point.
    int currPoint;

    public PurePursuitPath(MecanumHardware robot) {
        this(robot, new LinkedList<>());
    }

    public PurePursuitPath(MecanumHardware robot, Point ... points) {
        this(robot, Arrays.asList(points));
    }

    public PurePursuitPath(MecanumHardware robot, List<Point> waypoints) {
        this.waypoints = waypoints;
        this.currPoint = 0;
        this.robot = robot;
    }

    public void update() {
        // Get perpendicular bisector to line in question
        Pose robotPosition = robot.pose();

        // If we're following a line segment (not on final point):
        if (currPoint < waypoints.size() - 1) {
            System.out.println("Following line segment " + currPoint + " from " +
                    waypoints.get(currPoint).toString() + " to " + waypoints.get(currPoint + 1).toString());

            System.out.println("Robot is at position " + robot.pose().toString());

            Line currSegment = new Line(waypoints.get(currPoint), waypoints.get(currPoint + 1));
            Point center = currSegment.nearestLinePoint(robotPosition);

            System.out.println("Circle center is on point " + center.toString());

            // If we might need to shift to next line segment
            if (currPoint < waypoints.size() - 2) {
                List<Point> intersectionsWithNextLine = MathUtil.lineSegmentCircleIntersection(
                        waypoints.get(currPoint + 1), waypoints.get(currPoint + 2),
                        center, LOOK_AHEAD_DISTANCE
                );
                System.out.println("Intersections with next line: " + intersectionsWithNextLine.size());
                // If our circle intersects the next line, advance to next segment and try again
                if (intersectionsWithNextLine.size() > 0) {
                    currPoint++;
                    update();
                    return;
                }
            } else if (currPoint == waypoints.size() - 1){
                // Check if we're in circle
                if (waypoints.get(waypoints.size() - 1).distance(robotPosition) <= LOOK_AHEAD_DISTANCE) {
                    currPoint++;
                    update();
                    return;
                }
            }
            List<Point> intersectionsWithCurrentLine = MathUtil.lineSegmentCircleIntersection(
                    waypoints.get(currPoint), waypoints.get(currPoint + 1),
                    center, LOOK_AHEAD_DISTANCE
            );
            System.out.println("Intersections with current line: " + intersectionsWithCurrentLine.size());
            System.out.println("Closest intersection: " + intersectionsWithCurrentLine.get(0));
            if (intersectionsWithCurrentLine.size() == 0) {
                throw new ArrayIndexOutOfBoundsException("Target line segment not seen in look ahead distance");
            }
            robot.setPowers(MecanumPurePursuitController.goToPosition(robotPosition,
                    new Pose(intersectionsWithCurrentLine.get(0), 0), 1.0, 1.0));
        } else {
            robot.setPowers(MecanumPurePursuitController.goToPosition(robotPosition,
                    (Pose) waypoints.get(waypoints.size() - 1), 1.0, 1.0));
        }
    }
}
