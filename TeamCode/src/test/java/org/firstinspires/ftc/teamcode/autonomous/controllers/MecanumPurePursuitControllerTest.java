package org.firstinspires.ftc.teamcode.autonomous.controllers;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class MecanumPurePursuitControllerTest {

    final static Pose ORIGIN = new Pose(0, 0, 0);

    @Test
    void goToPositionStatic() {
        // First, ensure this works for simple cases of YEETing

        // Yeet far forwards
        assertEquals(
                new MecanumPowers(1, 0, 0),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new Waypoint(100, 0, -1),
                        null
                )
        );

        // Yeet forwards at half speed
        assertEquals(
                new MecanumPowers(0.5, 0, 0),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new Waypoint(6, 0, -1),
                        null
                )
        );

        // Yeet sideways and turning
        assertEquals(
                new MecanumPowers(0, 0.5, 0.5),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new HeadingControlledWaypoint(0, 6, -1, Math.PI/2),
                        null
                )
        );

        // Now, we'll make sure things are still accurate if the robot starts turned
        assertEquals(
                new MecanumPowers(-1.0, 0.5, -0.5),
                MecanumPurePursuitController.goToPosition(
                        new Pose(128, 128, Math.PI/2), ORIGIN,
                        new HeadingControlledWaypoint(128 - 6, 128 - 12, -1, 0),
                        null
                )
        );

        // Make sure automatic heading calculation works
        assertEquals(
                new MecanumPowers(0.5, 0.5,0.25),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new Waypoint(6, 6, -1),
                        null
                )
        );
    }

    @Test
    void goToStoppedPosition() {
        // SLIDE into place with our heading
        assertEquals(
                new MecanumPowers(0, 0,0),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, new Pose(0, 0, 4),
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(0, 0, -1, 2, -1)
                )
        );

        // SLIDE into place with our velocity
        assertEquals(
                new MecanumPowers(0, 0.5,0),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, new Pose(30, 0, 0),
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(6, 6, -1, 0, -1)
                )
        );

        // SLIDE into place with our velocity, even when at an angle
        assertEquals(
                new MecanumPowers(0, 0,0),
                MecanumPurePursuitController.goToPosition(
                        new Pose(0, 0, Math.PI/2), new Pose(0, 30, 0),
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(0, 6, -1, Math.PI/2, -1)
                )
        );

        // SLIDE into place with our velocity, even when at an angle, but only on X
        assertEquals(
                new MecanumPowers(0, -1,0),
                MecanumPurePursuitController.goToPosition(
                        new Pose(0, 0, -Math.PI/2), new Pose(0, -30, 0),
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(-12, -6, -1, -Math.PI/2, -1)
                )
        );
    }

    @Test
    void goToScootPosition() {
        // Six tests to verify directionality is correct

        assertEquals(
                new MecanumPowers(MecanumPurePursuitController.ONE_IN_AWAY_POWER, 0,0),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(1, 0, -1, 0, -1)
                )
        );

        assertEquals(
                new MecanumPowers(-MecanumPurePursuitController.ONE_IN_AWAY_POWER, 0,0),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(-1, 0, -1, 0, -1)
                )
        );

        assertEquals(
                new MecanumPowers(0, MecanumPurePursuitController.ONE_IN_AWAY_POWER, 0),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(0, 1, -1, 0, -1)
                )
        );

        assertEquals(
                new MecanumPowers(0, -MecanumPurePursuitController.ONE_IN_AWAY_POWER, 0),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(0, -1, -1, 0, -1)
                )
        );

        assertEquals(
                new MecanumPowers(0, 0, 0.25),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(0, 0, -1, 0.1, -1)
                )
        );

        assertEquals(
                new MecanumPowers(0, 0, -0.25),
                MecanumPurePursuitController.goToPosition(
                        ORIGIN, ORIGIN,
                        new Waypoint(-1, -1, -1),
                        new StopWaypoint(0, 0, -1, -0.1, -1)
                )
        );
    }
}