package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.DepositUntilSuccessful;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.FirstBlockDepositStop;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.FoundationGrabBackupPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.GrabBlockOptionallyRejectDouble;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.JoltsUntilBlockGrab;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.OptionalSkipFourthBlock;
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

    public static double FIRST_PLUNGE_TARGET_Y = 22;
    public static double BACK_PLUNGE_TARGET_X = -FIELD_RADIUS + 13;

    public static double UPPER_FRONT_PLUNGE_TARGET_X = -36 + 16;
    public static double MIDDLE_FRONT_PLUNGE_TARGET_X = -36 + 8;
    public static double LOWER_FRONT_PLUNGE_TARGET_X = -36;
    public static double LIFT_MAX_POWER = 0.6;

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
                new Waypoint(DEFAULT_START_POSITION, 4),

                // We want to move in strictly on the y-axis WRT the field to grab block,
                // which means robot must be at 45 degrees
                new HeadingControlledWaypoint(BACK_PLUNGE_TARGET_X + skystoneOffset, 48, 4, -0.75 * Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(BACK_PLUNGE_TARGET_X + skystoneOffset, FIRST_PLUNGE_TARGET_Y, 4, -0.75 * Math.PI,
                        3, new JoltsUntilBlockGrab(joltDirection)),

                new HeadingControlledWaypoint(BACK_PLUNGE_TARGET_X + skystoneOffset, 40, 10, -0.75 * Math.PI),

                // Now make our move to deposit
                new HeadingControlledWaypoint(-20, 41, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                new HeadingControlledWaypoint(18, 41, 12, Math.PI, Subroutines.SET_FOUNDATION_LATCHES_OUT),
                new HeadingControlledWaypoint(GRAB_FOUNDATION_LOCATION.x, 40, 8, Math.PI * 0.5),
                GRAB_FOUNDATION_LOCATION,
                new HeadingControlledWaypoint(20, 41, 6, Math.PI,  new FirstBlockDepositStop()),
                new HeadingControlledWaypoint(0, 41, 6, Math.PI),
                new HeadingControlledWaypoint(frontPlungeTarget + 16, 41, 6, Math.PI, Subroutines.ENABLE_INTAKE),

                new HeadingControlledWaypoint(frontPlungeTarget + 8, 38, 6, Math.toRadians(225)),
                new StopWaypoint(frontPlungeTarget - 1.5, 28.5, 4, Math.toRadians(225),
                        3, new JoltsUntilBlockGrab(joltDirection)),
                new HeadingControlledWaypoint(frontPlungeTarget + 4, 41, 12, Math.PI),
                // Now make our move to deposit
                new HeadingControlledWaypoint(-4, 43, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_INTAKED_BLOCK_AND_LIFT)),
                new HeadingControlledWaypoint(0, 43, 12, Math.PI),
                new StopWaypoint(16, 41, 8,
                        Math.PI, 8, new DepositUntilSuccessful(MecanumUtil.STOP))
        );

        /* What we do now depends on what blocks we grabbed first, since if we grabbed
        SkystoneState.UPPER the two skystones closest to the field wall haven't been touched.
         */

        if (SKYSTONE == SkystoneState.LOWER) {
            scoreSkystones.addAll(Waypoint.collate(
                    new HeadingControlledWaypoint(16, 43, 6, Math.PI),
                    new HeadingControlledWaypoint(0, 43, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                    new HeadingControlledWaypoint(-6, 43, 8, Math.toRadians(225), Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-25, 22, 8, Math.toRadians(225), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-53, 22, 8, Math.PI, Subroutines.CHECK_BLOCK_GRAB),

                    new Waypoint(-28, 43, 8),
                    new HeadingControlledWaypoint(-12, 43, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_INTAKED_BLOCK_AND_LIFT)),
                    new HeadingControlledWaypoint(0, 43, 8, Math.PI, Subroutines.SET_FANGS_UP)
            ));
        } else if (SKYSTONE == SkystoneState.MIDDLE || SKYSTONE == SkystoneState.UPPER) {
            scoreSkystones.addAll(Waypoint.collate(
                    new HeadingControlledWaypoint(-32, 43, 12, Math.PI, Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-42, 43, 6, Math.PI, Subroutines.ENABLE_INTAKE),

                    new HeadingControlledWaypoint(-50, 43, 6, Math.toRadians(225), Subroutines.CHECK_BLOCK_GRAB),
                    new StopWaypoint(-58, 25, 4, Math.toRadians(225),
                            3, new JoltsUntilBlockGrab(MecanumUtil.FORWARD)),
                    new HeadingControlledWaypoint(-54, 43, 12, Math.PI),
                    new HeadingControlledWaypoint(-12, 43, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_INTAKED_BLOCK_AND_LIFT))
            ));
        }

        scoreSkystones.addAll(Waypoint.collate(
                new StopWaypoint(16, 43, 8,
                        Math.PI, 8, new DepositUntilSuccessful(MecanumUtil.STOP))
        ));

        double startSwoop = -6;
        if (SKYSTONE == SkystoneState.LOWER) { // If we already did a swoop, shift it down
            startSwoop = -18;
        }

        /* Now we just do all that again, but with the fangs down */
        scoreSkystones.addAll(Waypoint.collate(
                new HeadingControlledWaypoint(15, 43, 6, Math.PI),
                new HeadingControlledWaypoint(0, 43, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                new HeadingControlledWaypoint(startSwoop, 43, 8, Math.toRadians(205), Subroutines.ENABLE_INTAKE),
                new HeadingControlledWaypoint(startSwoop - 19, 22, 8, Math.toRadians(205), Subroutines.CHECK_BLOCK_GRAB),
                new HeadingControlledWaypoint(-57, 22, 8, Math.PI, Subroutines.CHECK_BLOCK_GRAB),

                new Waypoint(-28, 43, 8),
                new HeadingControlledWaypoint(-12, 43, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_INTAKED_BLOCK_AND_LIFT)),
                new HeadingControlledWaypoint(0, 43, 8, Math.PI, Subroutines.SET_FANGS_UP),
                new StopWaypoint(36, 43, 8,
                        Math.PI, 8, new DepositUntilSuccessful(new MecanumPowers(-0.3, 0, 0), true))
        ));

        scoreSkystones.addAll(Waypoint.collate(
                new HeadingControlledWaypoint(44, 43, 6, Math.PI),
                new StopWaypoint(5, 43, 8, Math.PI, 0.01)
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
