package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.controllers.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.PointTurnWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.math.Line;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Point;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class PurePursuitPath {
    public static double TRACK_SPEED = 0.5;
    public static double DEAD_MAN_SWITCH = 2000;
    private SkystoneHardware robot;
    public List<Waypoint> waypoints;

    // currPoint in 0..n-2 means we're on the path from waypoints[currPoint] to
    // waypoints[currPoint + 1]. currPoint = n-1 means we're done.
    public int currPoint;
    boolean interrupting;
    public ElapsedTime timeUntilDeadman;

    public PurePursuitPath(SkystoneHardware robot) {
        this(robot, new LinkedList<>());
    }

    public PurePursuitPath(SkystoneHardware robot, Waypoint... points) {
        this(robot, Arrays.asList(points));
    }

    public PurePursuitPath(SkystoneHardware robot, List<Waypoint> waypoints) {
        // We need to deep copy our linked list so the same point doesn't get flipped multiple times
        this.waypoints = new LinkedList<>();
        for (Waypoint w : waypoints) {
            this.waypoints.add(w.clone());
        }

        this.currPoint = 0;
        this.robot = robot;
        this.interrupting = false;
        this.timeUntilDeadman = new ElapsedTime();

        if (!(waypoints.get(waypoints.size() - 1) instanceof StopWaypoint)) {
            throw new IllegalArgumentException("Final Pure Pursuit waypoint must be a StopWaypoint!");
        }
    }

    public void reverse() {
        for (Waypoint w : this.waypoints) {
            w.y = -w.y;

            // We also need to invert headings.
            if (w instanceof HeadingControlledWaypoint) {
                HeadingControlledWaypoint hCW = (HeadingControlledWaypoint) w;
                hCW.targetHeading = MathUtil.angleWrap(-hCW.targetHeading);
            }
        }
    }

    public void update() {
        Pose robotPosition = robot.pose();
        Pose robotVelocity = robot.localizer.relVelocity();
        // Note - our currPoint will only be the last point in the list once we're done moving
        // the robot

        // Before we do anything else, check if we're being interrupted
        if (interrupting) {
            boolean advance = ((Subroutines.ArrivalInterruptSubroutine) waypoints.get(currPoint).action).runCycle(robot);
            if (advance) {
                interrupting = false;
            } else {
                return; // Don't do anything else this cycle
            }
        }

        // Check whether we should advance to the next piece of the curve
        boolean jumpToNextSegment;
        do {
            jumpToNextSegment = false;
            Waypoint target = waypoints.get(currPoint + 1);

            // Stop waypoint deadman switch
            if (target instanceof StopWaypoint && timeUntilDeadman.milliseconds() > DEAD_MAN_SWITCH) {
                jumpToNextSegment = true;
            } else if (!(target instanceof StopWaypoint) || robot.localizer.relVelocity().radius() > 1) {
                timeUntilDeadman.reset();
            }
            if (target instanceof StopWaypoint) {
                if (robotPosition.distance(target) < ((StopWaypoint) target).allowedPositionError) {
                    jumpToNextSegment = true;
                }
            } else if (target instanceof PointTurnWaypoint) {
                PointTurnWaypoint ptTarget = (PointTurnWaypoint) target;
                if (Math.abs(robotPosition.heading - ptTarget.targetHeading) < ptTarget.allowedHeadingError) {
                    jumpToNextSegment = true;
                }
            } else {
                if (robotPosition.distance(target) < target.followDistance) {
                    jumpToNextSegment = true;
                }
            }

            // Run repeated subroutines, and see if they return true
            Subroutines.Subroutine action = waypoints.get(currPoint + 1).action;
            if (action instanceof Subroutines.RepeatedSubroutine) {
                if (((Subroutines.RepeatedSubroutine) action).runLoop(robot)) {
                    jumpToNextSegment = true;
                }
            }

            if (jumpToNextSegment) {
                currPoint++;
                action = waypoints.get(currPoint).action;
                if (action instanceof Subroutines.OnceOffSubroutine) {
                    ((Subroutines.OnceOffSubroutine) action).runOnce(robot);
                } else if (action instanceof Subroutines.MetaSubroutine) {
                    ((Subroutines.MetaSubroutine) action).runOnce(this, robot);
                }
                if (action instanceof Subroutines.ArrivalInterruptSubroutine) {
                    interrupting = true;
                    // TODO make code less gross by either not using or committing to recursion
                    this.update();
                    return;
                }

            }
        } while (jumpToNextSegment && currPoint < waypoints.size() - 1);
        if (finished()) {return;}

        Waypoint target = waypoints.get(currPoint + 1);
        // If we're making a stop and in the stop portion of the move
        if (target instanceof StopWaypoint && robotPosition.distance(target) < target.followDistance) {
            robot.setPowers(MecanumPurePursuitController.goToPosition(
                    robotPosition, robotVelocity, target, TRACK_SPEED, false));
            System.out.println("Locking onto point " + target.toString());
        } else if (target instanceof PointTurnWaypoint) {
            robot.setPowers(MecanumPurePursuitController.goToPosition(
                    robotPosition, robotVelocity, target, 1.0, true));
        } else {
            trackToLine(
                    robotPosition, robotVelocity,
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
    private void trackToLine(Pose robotPosition, Pose robotVelocity, Waypoint start, Waypoint mid) {
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
        robot.setPowers(MecanumPurePursuitController.goToPosition(robotPosition, robotVelocity,
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
