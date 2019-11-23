package org.firstinspires.ftc.teamcode.autonomous.controllers;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Point;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;

public class MecanumPurePursuitController {
    // How far we slip if we're moving with max power in each of these directions
    public static Pose SLIP_DISTANCES = new Pose(22, 5, Math.toRadians(30));
    //public static Pose SLIP_DISTANCES = new Pose(1, 1, Math.toRadians(1));
    public static Pose MAX_VELOCITY = new Pose(48, 60, 5); // in/sec
    public static double MIN_TRANSLATION_POWERS = 0.3;
    public static double CUTOFF_MOTOR_POWER = 0.05;
    public static Pose GUNNING_REDUCTION_DISTANCES = new Pose(6, 6, Math.PI/2);
    public static Pose FINE_REDUCTION_DISTANCES = new Pose(30, 30, Math.PI);


    public static MecanumPowers goToPosition(Pose robotPose, Pose robotVelocity, Waypoint target, double movementSpeed, boolean gunning) {
        //System.out.println(target);
        //System.out.println(robotPose);

        // Sometimes we need to move accurately to a position, while many other times we just have
        // to get "about" somewhere - our waypoints are approximations anyway. If we need to be exact

        Pose relSlip = SLIP_DISTANCES.multiply(robotVelocity.divideBy(MAX_VELOCITY));

        double distance = target.minus(robotPose).radius();
        double relAngle = robotPose.minus(target).atan() - robotPose.heading;
        double relX = distance * Math.cos(relAngle) + relSlip.x;
        double relY = distance * Math.sin(relAngle) + relSlip.y;

        // We negate x and y power because we want to move in the opposite direction of our error
        Pose translationPowers = new Pose(-relX, -relY, 0).scale(movementSpeed);

        Pose reductionDistances = gunning ? GUNNING_REDUCTION_DISTANCES : FINE_REDUCTION_DISTANCES;
        translationPowers.x /= reductionDistances.x;
        translationPowers.y /= reductionDistances.y;

        // Ensure we're moving x/y a little unless we're stopped
        if (translationPowers.radius() < MIN_TRANSLATION_POWERS) {
            translationPowers.scale(MIN_TRANSLATION_POWERS / translationPowers.radius());
        }

        // Heading always wants to stop at a point, so we'll treat this the same regardless if we're
        // at a stop waypoint or a normal one
        double forwardAngle = target.minus(robotPose).atan();
        double backwardAngle = forwardAngle + Math.PI;
        double angleToForward = MathUtil.angleWrap(forwardAngle - robotPose.heading);
        double angleToBackward = MathUtil.angleWrap(backwardAngle - robotPose.heading);
        double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
        double desiredAngle = target instanceof HeadingControlledWaypoint ?
                ((HeadingControlledWaypoint) target).targetHeading : autoAngle;

        // We'll do a quadratic decay to zero to only correct for really big heading errors
        double angleToTarget = MathUtil.angleWrap(desiredAngle - robotPose.heading);
        translationPowers.heading = angleToTarget / reductionDistances.heading;
        // We're going to square the turn power to reduce it when it is low
        //translationPowers.heading = MathUtil.powRetainingSign(angleToTarget / REDUCTION_DISTANCES.heading, 1);
        return new MecanumPowers(translationPowers);
    }
}
