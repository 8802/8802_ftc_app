package org.firstinspires.ftc.teamcode.autonomous.controllers;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;

@Config
public class MecanumPurePursuitController {
    // How far we slip if we're moving 1 in/sec (or 1 rad/sec) in each of these directions
    public static Pose SLIP_DISTANCES = new Pose(1.5, 0, 0);
    public static double UNDERSHOOT_DIST = 2; // Aim to stop 2 in away from target, and use small motions to finish it
    public static double MIN_SLIP_SPEED = 8;
    public static Pose GUNNING_REDUCTION_DISTANCES = new Pose(12, 12, Math.PI);
    public static Pose ONE_AWAY_POWERS = new Pose(0.08, 0.12, 0.1);
    public static double CLOSE_EXPONENT = 1.0/6.0;


    private static Pose relDistanceToTarget(Pose robot, Waypoint target) {
        double distance = target.minus(robot).radius();
        double relAngle = robot.minus(target).atan() - robot.heading;
        double relX = distance * Math.cos(relAngle);
        double relY = distance * Math.sin(relAngle);
        return new Pose(relX, relY, relAngle);
    }

    public static MecanumPowers goToPosition(Pose robotPose, Pose robotVelocity, Waypoint target, StopWaypoint finalTarget) {

        // Sometimes we need to move accurately to a position, while many other times we just have
        // to get "about" somewhere - our waypoints are approximations anyway. We'll assume we only
        // need to be exact if we're stopping somewhere - otherwise we can increase accuracy by just
        // using a shorter look ahead distance. If finalTarget is set, we'll try to finely adjust
        // speed and slippage to hit that point. Otherwise, we'll just YEET over there.

        System.out.println("Current velocity: " + robotVelocity.toString());
        if (finalTarget == null || robotPose.distance(finalTarget) > 18) {
            Pose relPPTarget = relDistanceToTarget(robotPose, target);

            // We negate x and y power because we want to move in the opposite direction of our error
            Pose translationPowers = relPPTarget.scale(-1).divideBy(GUNNING_REDUCTION_DISTANCES);
            System.out.println(translationPowers.toString());

            // Heading always wants to stop at a point, so we'll treat this the same regardless if we're
            // at a stop waypoint or a normal one
            double forwardAngle = target.minus(robotPose).atan();
            double backwardAngle = forwardAngle + Math.PI;
            double angleToForward = MathUtil.angleWrap(forwardAngle - robotPose.heading);
            double angleToBackward = MathUtil.angleWrap(backwardAngle - robotPose.heading);
            double autoAngle = Math.abs(angleToForward) < Math.abs(angleToBackward) ? forwardAngle : backwardAngle;
            double desiredAngle = target instanceof HeadingControlledWaypoint ?
                    ((HeadingControlledWaypoint) target).targetHeading : autoAngle;

            double angleToTarget = MathUtil.angleWrap(desiredAngle - robotPose.heading);
            translationPowers.heading = angleToTarget / GUNNING_REDUCTION_DISTANCES.heading;
            return new MecanumPowers(translationPowers);
        } else if (robotVelocity.radius() > MIN_SLIP_SPEED) { // If we're moving more than 8 in/sec and we're close to our target
            StopWaypoint t = finalTarget.clone();
            t.y += 6;
            // We're approaching a point, and we need to not overshoot
            Pose relVelocity = new Pose(robotVelocity.rotated(-robotPose.heading), robotVelocity.heading);
            System.out.println(relVelocity);
            Pose relSlipDistances = relVelocity.multiply(SLIP_DISTANCES);
            System.out.println(relSlipDistances);
            Pose relAbsTarget = relDistanceToTarget(robotPose, t).add(relSlipDistances);
            System.out.println(relAbsTarget);
            // We negate this here so our negation in translationPowers is cancelled out
            relAbsTarget.heading = -MathUtil.angleWrap(t.targetHeading - robotPose.heading - relSlipDistances.heading);

            Pose translationPowers = relAbsTarget.scale(-1).divideBy(GUNNING_REDUCTION_DISTANCES);
            return new MecanumPowers(translationPowers);
        } else {
            // Now we just need to nudge the robot. We'll hold our heading with a simple P-loop,
            // and adjust our position with a special function
            Pose relAbsTarget = relDistanceToTarget(robotPose, finalTarget);
            double angleToTarget = MathUtil.angleWrap(finalTarget.targetHeading - robotPose.heading);

            // Now, we're going to use the polynomial function x^1/6 to compute our powers
            Pose dirPowers = new Pose(
                    -MathUtil.powRetainingSign(relAbsTarget.x, CLOSE_EXPONENT),
                    -MathUtil.powRetainingSign(relAbsTarget.y, CLOSE_EXPONENT),
                    MathUtil.powRetainingSign(angleToTarget, CLOSE_EXPONENT)
            );
            return new MecanumPowers(dirPowers.multiply(ONE_AWAY_POWERS));
        }
    }
}
