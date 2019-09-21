package org.firstinspires.ftc.teamcode.autonomous.controllers;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;

public class MecanumPurePursuitController {
    public static double REDUCE_TURN_SPEED_THRESHOLD = Math.PI/8;
    public static double REDUCE_MOVE_SPEED_THRESHOLD = 6;

    public static MecanumPowers goToPosition(Pose robotPose, Waypoint target, double movementSpeed, double point_speed) {
        //System.out.println(target);
        //System.out.println(robotPose);

        double distance = target.minus(robotPose).radius();
        double relAngle = robotPose.minus(target).atan() - robotPose.heading;
        double relX = distance * Math.cos(relAngle);
        double relY = distance * Math.sin(relAngle);

        // We negate x and y power because we want to move in the opposite direction of our error
        double xPower = movementSpeed * -relX / REDUCE_MOVE_SPEED_THRESHOLD;
        double yPower = movementSpeed * -relY / REDUCE_MOVE_SPEED_THRESHOLD;

        // Get heading that points from robot to target point. We want this to be the robot's heading
        // TODO support driving backwards by changing target.minus(robotPose).atan()
        double desiredAngle = target instanceof HeadingControlledWaypoint ?
                ((HeadingControlledWaypoint) target).targetHeading :
                target.minus(robotPose).atan();

        double angleToTarget = MathUtil.angleWrap(desiredAngle - robotPose.heading);
        double turnPower = angleToTarget / REDUCE_TURN_SPEED_THRESHOLD;
        return new MecanumPowers(xPower, yPower, turnPower);
    }
}
