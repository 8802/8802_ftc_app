package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.CloseDepositUntilSuccessful;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.DepositUntilSuccessful;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.FirstBlockDepositStop;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.FoundationGrabBackupPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.GrabBlockOptionallyRejectDouble;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.JoltsUntilBlockGrab;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.common.elements.SkystoneState;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;

import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware.FIELD_RADIUS;

// Import constants

@Config
public class SSAutoMovingFoundationRed extends PurePursuitAutoRed {

    Waypoint GRAB_FOUNDATION_LOCATION = new StopWaypoint(FIELD_RADIUS - 5 - (34.5/2), 34,
            6, Math.PI * 0.5, 4, new FoundationGrabBackupPath());

    public static double UPPER_FRONT_PLUNGE_TARGET_X = -36 + 16;
    public static double MIDDLE_FRONT_PLUNGE_TARGET_X = -36 + 8;
    public static double LOWER_FRONT_PLUNGE_TARGET_X = -36;
    public static double LIFT_MAX_POWER = 1;

    @Override
    public Pose getBlueStartPosition() {
        return DEFAULT_START_POSITION;
    }

    @Override
    public List<Waypoint> getPurePursuitWaypoints() {
        MecanumPowers joltDirection = (ALLIANCE == Alliance.BLUE) ? MecanumUtil.FORWARD_RIGHT : MecanumUtil.FORWARD_LEFT;
        double skystoneOffset = SKYSTONE.index * 8;

        // Pick our front plunge target
        double frontPlungeTarget = -1;
        if (SKYSTONE == SkystoneState.UPPER) {
            frontPlungeTarget = UPPER_FRONT_PLUNGE_TARGET_X;
        } else if (SKYSTONE == SkystoneState.MIDDLE) {
            frontPlungeTarget = MIDDLE_FRONT_PLUNGE_TARGET_X;
        } else if (SKYSTONE == SkystoneState.LOWER) {
            frontPlungeTarget = LOWER_FRONT_PLUNGE_TARGET_X;
        }

        LinkedList<Waypoint> scoreSkystones = Waypoint.collate(
                new Waypoint(DEFAULT_START_POSITION, 4)
        );
        if (SKYSTONE == SkystoneState.LOWER) {
            scoreSkystones.addAll(Waypoint.collate(
                    new Waypoint(DEFAULT_START_POSITION.x, 55, 10),
                    new StopWaypoint(-FIELD_RADIUS + 13, 25, 6, -0.75 * Math.PI,
                            3, new JoltsUntilBlockGrab(joltDirection)),
                    new HeadingControlledWaypoint(-FIELD_RADIUS + 13, 35, 6, -0.75 * Math.PI),
                    new HeadingControlledWaypoint(-FIELD_RADIUS + 21, 36, 10, -0.75 * Math.PI)
            ));
        } else if (SKYSTONE == SkystoneState.MIDDLE) {
            scoreSkystones.addAll(Waypoint.collate(
                    new Waypoint(DEFAULT_START_POSITION.x, 55, 10),
                    new StopWaypoint(-FIELD_RADIUS + 21, 25, 6, -0.75 * Math.PI,
                            3, new JoltsUntilBlockGrab(joltDirection)),
                    new HeadingControlledWaypoint(-FIELD_RADIUS + 21, 35, 6, -0.75 * Math.PI),
                    new HeadingControlledWaypoint(-FIELD_RADIUS + 29, 36, 10, -0.75 * Math.PI)
            ));
        } else {
            scoreSkystones.addAll(Waypoint.collate(
                    new HeadingControlledWaypoint(DEFAULT_START_POSITION.x, 45, 10, -Math.PI/2),
                    new HeadingControlledWaypoint(DEFAULT_START_POSITION.x, 34, 6, -0.75 * Math.PI),
                    new StopWaypoint(-39, 25, 5, -0.75 * Math.PI,
                            4, new JoltsUntilBlockGrab(joltDirection)),
                    new HeadingControlledWaypoint(-FIELD_RADIUS + 29, 35, 6, -0.75 * Math.PI),
                    new HeadingControlledWaypoint(-FIELD_RADIUS + 37, 36, 10, -0.75 * Math.PI)
            ));
        }

        scoreSkystones.addAll(Waypoint.collate(
                        // Now make our move to deposit
                new HeadingControlledWaypoint(-20, 41, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                new HeadingControlledWaypoint(18, 41, 12, Math.PI, Subroutines.SET_FOUNDATION_LATCHES_OUT),
                new HeadingControlledWaypoint(GRAB_FOUNDATION_LOCATION.x, 38, 8, Math.PI * 0.5),
                GRAB_FOUNDATION_LOCATION,
                new HeadingControlledWaypoint(20, 41, 8, Math.PI, new FirstBlockDepositStop()),
                new HeadingControlledWaypoint(8, 41, 10, Math.PI, Subroutines.SET_FOUNDATION_LATCHES_UP),
                new HeadingControlledWaypoint(frontPlungeTarget + 16, 36, 6, Math.PI, Subroutines.ENABLE_INTAKE),

                new HeadingControlledWaypoint(frontPlungeTarget + 8, 36, 6, Math.toRadians(225)),
                new StopWaypoint(frontPlungeTarget - 1.5, 26.5, 4, Math.toRadians(225),
                        3, new JoltsUntilBlockGrab(joltDirection)),
                new HeadingControlledWaypoint(frontPlungeTarget + 4, 41, 12, Math.PI),
                // Now make our move to deposit
                new HeadingControlledWaypoint(-4, 43, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                new HeadingControlledWaypoint(0, 43, 12, Math.PI),
                new StopWaypoint(12, 41, 8,
                        Math.PI, -1, new CloseDepositUntilSuccessful())
        ));

        /* What we do now depends on what blocks we grabbed first, since if we grabbed
        SkystoneState.UPPER the two skystones closest to the field wall haven't been touched.
         */
        if (SKYSTONE == SkystoneState.UPPER) {
            scoreSkystones.addAll(Waypoint.collate(
                    /* Third block goes for 2cd lowest block */
                    new HeadingControlledWaypoint(-24, 36, 8, Math.PI, Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-40, 30, 8, Math.toRadians(205)),
                    new StopWaypoint(-50, 24, 4, Math.toRadians(205),
                            3),
                    new HeadingControlledWaypoint(-46, 36, 12, Math.PI),
                    new HeadingControlledWaypoint(-12, 41, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new StopWaypoint(12, 40, 8, // TOOD verify this 40 actually works better than 35
                            Math.PI, -1, new CloseDepositUntilSuccessful()),

                    /* Fourth block goes for 1st lowest */
                    new HeadingControlledWaypoint(-32, 36, 8, Math.PI, Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-48, 30, 8, Math.toRadians(205)),
                    new StopWaypoint(-55, 25, 6, Math.toRadians(205),
                            4.5, new JoltsUntilBlockGrab(MecanumUtil.FORWARD)),
                    new HeadingControlledWaypoint(-54, 36, 12, Math.PI),
                    new HeadingControlledWaypoint(-12, 41, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 46, 12, Math.PI),
                    new StopWaypoint(50, 46, 8, Math.PI, -1, new CloseDepositUntilSuccessful()),

                    /* Fifth block is a swooping pattern */
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.toRadians(225), Subroutines.SET_FANGS_DOWN),
                    new HeadingControlledWaypoint(-36, 12, 5, Math.toRadians(225), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-16, 39, 12, Math.toRadians(225)),
                    new HeadingControlledWaypoint(-4, 39, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 41, 12, Math.PI),
                    new StopWaypoint(50, 48, 8,
                            Math.PI, -1, new DepositUntilSuccessful())
            ));
        } else if (SKYSTONE == SkystoneState.MIDDLE) {
            scoreSkystones.addAll(Waypoint.collate(
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.toRadians(240), Subroutines.SET_RIGHT_FANG_DOWN),
                    new HeadingControlledWaypoint(-30, 12, 4.5, Math.toRadians(240), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-16, 36, 12, Math.toRadians(240)),
                    new HeadingControlledWaypoint(-4, 41, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new StopWaypoint(12, 41, 8, // TOOD verify this 40 actually works better than 35
                            Math.PI, -1, new CloseDepositUntilSuccessful()),

                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-25, 39, 8, Math.toRadians(225), Subroutines.SET_RIGHT_FANG_DOWN),
                    new HeadingControlledWaypoint(-49, 12, 4.5, Math.toRadians(225), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-25, 36, 12, Math.toRadians(225)),
                    new HeadingControlledWaypoint(-4, 41, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new StopWaypoint(50, 41, 8, // TOOD verify this 40 actually works better than 35
                            Math.PI, -1, new CloseDepositUntilSuccessful()),

                    new HeadingControlledWaypoint(-32, 36, 8, Math.PI, Subroutines.FANGS_DOWN_AND_INTAKE),
                    new HeadingControlledWaypoint(-48, 30, 8, Math.toRadians(205), Subroutines.LIFT_FANGS_CHECK_BLOCK_GRAB),
                    new StopWaypoint(-55, 25, 6, Math.toRadians(205),
                            4.5, new JoltsUntilBlockGrab(MecanumUtil.FORWARD)),
                    new HeadingControlledWaypoint(-12, 39, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 41, 12, Math.PI),
                    new StopWaypoint(50, 41, 8, Math.PI, -1, new DepositUntilSuccessful())
            ));
        } else {
            scoreSkystones.addAll(Waypoint.collate(
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-10, 39, 6, Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                    new StopWaypoint(-18, 26.5, 4, Math.toRadians(225),
                            3, new JoltsUntilBlockGrab(joltDirection)),
                    new HeadingControlledWaypoint(-16, 39, 12, Math.PI),
                    new HeadingControlledWaypoint(-4, 39, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 41, 12, Math.PI),
                    new StopWaypoint(12, 40, 8,
                            Math.PI, -1, new CloseDepositUntilSuccessful()),

                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.toRadians(225), Subroutines.SET_FANGS_DOWN),
                    new HeadingControlledWaypoint(-36, 12, 6, Math.toRadians(225), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-16, 36, 12, Math.toRadians(225)),
                    new HeadingControlledWaypoint(-4, 36, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 41, 12, Math.PI),
                    new StopWaypoint(50, 41, 8,
                            Math.PI, -1, new CloseDepositUntilSuccessful()),

                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.FANGS_DOWN_AND_INTAKE),
                    new HeadingControlledWaypoint(-12, 39, 8, -0.75 * Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-30, 18, 8, -0.75 * Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-50, 18, 8, Math.PI, Subroutines.LIFT_FANGS_CHECK_BLOCK_GRAB),
                    new StopWaypoint(-56, 18, 8, Math.PI, 4, new JoltsUntilBlockGrab(MecanumUtil.FORWARD)),

                    new HeadingControlledWaypoint(-24, 33, 8, -0.85 * Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI),
                    new HeadingControlledWaypoint(0, 41, 12, Math.PI),
                    new StopWaypoint(50, 41, 8,
                            Math.PI, -1, new DepositUntilSuccessful())
            ));
        }
        scoreSkystones.addAll(Waypoint.collate(
                /* Drive back. This segment can be teleported to if time is running low */
                new HeadingControlledWaypoint(30, 41, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                new StopWaypoint(2, 41, 8, Math.PI, -1)
        ));

        return scoreSkystones;
    }

    @Override
    public void start() {
        super.start();
        robot.pidLift.lift.setMaxPower(LIFT_MAX_POWER);
        Subroutines.ENABLE_INTAKE.runOnce(robot);
    }
}
