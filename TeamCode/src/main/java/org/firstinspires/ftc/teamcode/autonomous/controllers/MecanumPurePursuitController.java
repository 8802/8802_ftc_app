package org.firstinspires.ftc.teamcode.autonomous.controllers;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;

public class MecanumPurePursuitController {
    public static double REDUCE_TURN_SPEED_THRESHOLD = Math.PI/2;
    public static double REDUCE_MOVE_SPEED_THRESHOLD = 30;

    public static double GUNNING_REDUCE_TURN_SPEED_THRESHOLD = Math.PI/8;
    public static double GUNNING_REDUCE_MOVE_SPEED_THRESHOLD = 6;

    public static MecanumPowers goToPosition(Pose robotPose, Waypoint target, double movementSpeed, boolean gunning) {
        //System.out.println(target);
        //System.out.println(robotPose);

        double reduceMoveSpeedThreshold = gunning ? GUNNING_REDUCE_MOVE_SPEED_THRESHOLD : REDUCE_MOVE_SPEED_THRESHOLD;
        double reduceTurnSpeedThreshold = gunning ? GUNNING_REDUCE_TURN_SPEED_THRESHOLD : REDUCE_TURN_SPEED_THRESHOLD;

        double distance = target.minus(robotPose).radius();
        double relAngle = robotPose.minus(target).atan() - robotPose.heading;
        double relX = distance * Math.cos(relAngle);
        double relY = distance * Math.sin(relAngle);

        // We negate x and y power because we want to move in the opposite direction of our error
        double xPower = movementSpeed * -relX / reduceMoveSpeedThreshold;
        double yPower = movementSpeed * -relY / reduceMoveSpeedThreshold;

        // Get heading that points from robot to target point. We want this to be the robot's heading
        double forwardAngle = target.minus(robotPose).atan();
        double backwardAngle = forwardAngle + Math.PI;
        double angleToForward = MathUtil.angleWrap(forwardAngle - robotPose.heading);
        double angleToBackward = MathUtil.angleWrap(backwardAngle - robotPose.heading);
        double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;

        System.out.println(target instanceof HeadingControlledWaypoint);

        double desiredAngle = target instanceof HeadingControlledWaypoint ?
                ((HeadingControlledWaypoint) target).targetHeading : autoAngle;

        double angleToTarget = MathUtil.angleWrap(desiredAngle - robotPose.heading);
        double turnPower = angleToTarget / reduceTurnSpeedThreshold;
        return new MecanumPowers(xPower, yPower, turnPower);
    }
}
