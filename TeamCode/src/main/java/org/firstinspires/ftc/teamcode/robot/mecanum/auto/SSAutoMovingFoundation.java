package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.DepositUntilSuccessful;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.FirstBlockDepositStop;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.FoundationGrabBackupPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.JoltsUntilBlockGrab;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.GrabBlockOptionallyRejectDouble;
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
public class SSAutoMovingFoundation extends PurePursuitAuto {

    Waypoint GRAB_FOUNDATION_LOCATION = new StopWaypoint(FIELD_RADIUS - 11 - (34.5/2), 30,
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
                    new StopWaypoint(-FIELD_RADIUS + 30.5, 25, 6, -0.75 * Math.PI,
                            3, new JoltsUntilBlockGrab(joltDirection)),
                    new HeadingControlledWaypoint(-FIELD_RADIUS + 29, 35, 6, -0.75 * Math.PI),
                    new HeadingControlledWaypoint(-FIELD_RADIUS + 37, 36, 10, -0.75 * Math.PI)
            ));
        }

        scoreSkystones.addAll(Waypoint.collate(
                        // Now make our move to deposit
                new HeadingControlledWaypoint(-20, 36, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                new HeadingControlledWaypoint(18, 38, 12, Math.PI, Subroutines.SET_FOUNDATION_LATCHES_OUT),
                new HeadingControlledWaypoint(GRAB_FOUNDATION_LOCATION.x, 38, 8, Math.PI * 0.5),
                GRAB_FOUNDATION_LOCATION,
                new HeadingControlledWaypoint(20, 36, 8, Math.PI, new FirstBlockDepositStop()),
                new HeadingControlledWaypoint(8, 36, 10, Math.PI, Subroutines.SET_FOUNDATION_LATCHES_UP),
                new HeadingControlledWaypoint(frontPlungeTarget + 16, 36, 6, Math.PI, Subroutines.ENABLE_INTAKE),

                new HeadingControlledWaypoint(frontPlungeTarget + 8, 36, 6, Math.toRadians(225)),
                new StopWaypoint(frontPlungeTarget - 1.5, 26.5, 4, Math.toRadians(225),
                        3, new JoltsUntilBlockGrab(joltDirection)),
                new HeadingControlledWaypoint(frontPlungeTarget + 4, 36, 12, Math.PI),
                // Now make our move to deposit
                new HeadingControlledWaypoint(-4, 36, 12, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                new HeadingControlledWaypoint(0, 36, 12, Math.PI),
                new StopWaypoint(12, 41, 8,
                        Math.PI, -1, new DepositUntilSuccessful())
        ));

        /* What we do now depends on what blocks we grabbed first, since if we grabbed
        SkystoneState.UPPER the two skystones closest to the field wall haven't been touched.
         */
        if (SKYSTONE == SkystoneState.UPPER) {
            scoreSkystones.addAll(Waypoint.collate(
                    /* Third block goes for 2cd lowest block */
                    new HeadingControlledWaypoint(-24, 36, 12, Math.PI, Subroutines.ENABLE_INTAKE),
                    new StopWaypoint(-48, 25, 6, Math.toRadians(205),
                            3, new JoltsUntilBlockGrab(MecanumUtil.FORWARD)),
                    new HeadingControlledWaypoint(-46, 36, 12, Math.PI),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new StopWaypoint(12, 34.5, 8,
                            Math.PI, -1, new DepositUntilSuccessful()),

                    /* Fourth block goes for 1st lowest */
                    new HeadingControlledWaypoint(-32, 36, 8, Math.PI, Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-48, 30, 8, Math.toRadians(205)),
                    new StopWaypoint(-55, 25, 6, Math.toRadians(205),
                            4.5, new JoltsUntilBlockGrab(MecanumUtil.FORWARD)),
                    new HeadingControlledWaypoint(-54, 36, 12, Math.PI),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new StopWaypoint(12, 40, 8, Math.PI, -1, new DepositUntilSuccessful()),

                    /* Fifth block is a swooping pattern */
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                    new HeadingControlledWaypoint(-6, 39, 8, Math.toRadians(205), Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-25, 20, 8, Math.toRadians(205), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-57, 20, 8, Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-28, 36, 8, Math.toRadians(205)),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_UP),
                    new StopWaypoint(36, 40, 8,
                            Math.PI, -1, new DepositUntilSuccessful(DepositUntilSuccessful.DepositHeight.HIGH)),

                    /* After pushing the foundation, drive back */
                    new HeadingControlledWaypoint(44, 39, 6, Math.PI),
                    new StopWaypoint(2, 39, 8, Math.PI, -1)
            ));
        } else if (SKYSTONE == SkystoneState.MIDDLE) {
            scoreSkystones.addAll(Waypoint.collate(
                    /* Third block goes for lowest block */
                    new HeadingControlledWaypoint(-32, 36, 12, Math.PI, Subroutines.ENABLE_INTAKE),
                    new StopWaypoint(-56, 25, 6, Math.toRadians(205),
                            3, new JoltsUntilBlockGrab(MecanumUtil.FORWARD)),
                    new HeadingControlledWaypoint(-54, 36, 12, Math.PI),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new StopWaypoint(12, 36, 8, Math.PI, -1, new DepositUntilSuccessful()),

                    /* Fourth block is a high swoop */
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                    new HeadingControlledWaypoint(-6, 39, 8, Math.toRadians(205), Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-25, 20, 8, Math.toRadians(205), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-57, 20, 8, Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                    new Waypoint(-28, 39, 8),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_UP),
                    new StopWaypoint(12, 36, 8, Math.PI, -1, new DepositUntilSuccessful()),

                    /* Fifth block is the same high swoop */
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                    new HeadingControlledWaypoint(-6, 39, 8, Math.toRadians(205), Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-25, 20, 8, Math.toRadians(205), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-57, 20, 8, Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                    new Waypoint(-28, 39, 8),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_UP),
                    new StopWaypoint(40, 36, 8,
                            Math.PI, -1, new DepositUntilSuccessful(DepositUntilSuccessful.DepositHeight.HIGH)),

                    /* After pushing the foundation, drive back */
                    new HeadingControlledWaypoint(44, 39, 6, Math.PI),
                    new StopWaypoint(2, 39, 8, Math.PI, -1)
            ));
        } else {
            scoreSkystones.addAll(Waypoint.collate(
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                    new HeadingControlledWaypoint(-6, 39, 8, Math.toRadians(205), Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-14, 20, 8, Math.toRadians(205), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-57, 20, 8, Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                    new Waypoint(-28, 39, 8),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_UP),
                    new StopWaypoint(12, 36, 8, Math.PI, -1, new DepositUntilSuccessful()),

                    /* Fourth block is a high swoop */
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                    new HeadingControlledWaypoint(-6, 39, 8, Math.toRadians(205), Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-25, 20, 8, Math.toRadians(205), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-57, 20, 8, Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                    new Waypoint(-28, 39, 8),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_UP),
                    new StopWaypoint(12, 36, 8, Math.PI, -1, new DepositUntilSuccessful()),

                    /* Fifth block is the same high swoop */
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                    new HeadingControlledWaypoint(-6, 39, 8, Math.toRadians(205), Subroutines.ENABLE_INTAKE),
                    new HeadingControlledWaypoint(-25, 20, 8, Math.toRadians(205), Subroutines.CHECK_BLOCK_GRAB),
                    new HeadingControlledWaypoint(-57, 20, 8, Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                    new Waypoint(-28, 39, 8),
                    new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                    new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_UP),
                    new StopWaypoint(40, 36, 8,
                            Math.PI, -1, new DepositUntilSuccessful(DepositUntilSuccessful.DepositHeight.HIGH)),

                    /* After pushing the foundation, drive back */
                    new HeadingControlledWaypoint(44, 39, 6, Math.PI),
                    new StopWaypoint(2, 39, 8, Math.PI, -1)
            ));
        }

        /*scoreSkystones.addAll(Waypoint.collate(
                new HeadingControlledWaypoint(16, 39, 8, Math.PI),
                new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_DOWN),
                new HeadingControlledWaypoint(-6, 39, 8, Math.toRadians(225), Subroutines.ENABLE_INTAKE),
                new HeadingControlledWaypoint(-25, 20, 8, Math.toRadians(225), Subroutines.CHECK_BLOCK_GRAB),
                new HeadingControlledWaypoint(-53, 20, 8, Math.PI, Subroutines.CHECK_BLOCK_GRAB),

                new Waypoint(-28, 39, 8),
                new HeadingControlledWaypoint(-12, 39, 8, Math.PI, new GrabBlockOptionallyRejectDouble(Subroutines.GRAB_BLOCK_NO_EXTEND)),
                new HeadingControlledWaypoint(0, 39, 8, Math.PI, Subroutines.SET_FANGS_UP)
        ));*/

        return scoreSkystones;
    }

    @Override
    public void start() {
        super.start();
        robot.pidLift.lift.setMaxPower(LIFT_MAX_POWER);
        Subroutines.ENABLE_INTAKE.runOnce(robot);
    }
}
