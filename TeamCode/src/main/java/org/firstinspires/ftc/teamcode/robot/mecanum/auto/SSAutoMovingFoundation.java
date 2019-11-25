package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.ActionAndWait;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.DepositUntilSuccessful;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.FoundationGrabBackupPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.IgnorantPointTurn;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.JoltsUntilBlockGrab;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.OptionallyRejectBlock;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.PointTurnWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.RamFoundationBackward;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.RamFoundationBackwardRed;
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

    StopWaypoint DEPOSIT_LOCATION = new StopWaypoint(FIELD_RADIUS - 30, 36, 8,
            Math.PI, 8, new ActionAndWait(1000, Subroutines.SMART_DROP_BLOCK));

    Waypoint GRAB_FOUNDATION_LOCATION = new StopWaypoint(FIELD_RADIUS - 10 - (34.5/2), 30,
            6, Math.PI * 0.5, 4, new FoundationGrabBackupPath());

    public static double PLUNGE_TARGET_Y = 22;
    public static double BACK_PLUNGE_TARGET_X = -FIELD_RADIUS + 13;
    public static double FRONT_PLUNGE_TARGET_X = -FIELD_RADIUS + 40;

    @Override
    public Pose getBlueStartPosition() {
        return DEFAULT_START_POSITION;
    }

    @Override
    public List<Waypoint> getPurePursuitWaypoints() {
        MecanumPowers joltDirection = (ALLIANCE == Alliance.BLUE) ? MecanumUtil.FORWARD_RIGHT : MecanumUtil.FORWARD_LEFT;
        double skystoneOffset = SKYSTONE.index * 8;

        LinkedList<Waypoint> scoreSkystones = Waypoint.collate(
                new Waypoint(DEFAULT_START_POSITION, 4),

                // We want to move in strictly on the y-axis WRT the field to grab block,
                // which means robot must be at 45 degrees
                new HeadingControlledWaypoint(BACK_PLUNGE_TARGET_X + skystoneOffset, 48, 4, -0.75 * Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(BACK_PLUNGE_TARGET_X + skystoneOffset, PLUNGE_TARGET_Y, 4, -0.75 * Math.PI,
                        3, new JoltsUntilBlockGrab(joltDirection)),
                new HeadingControlledWaypoint(BACK_PLUNGE_TARGET_X + skystoneOffset, 36, 12, -Math.PI),

                // Now make our move to deposit
                new HeadingControlledWaypoint(-8, 36, 12, Math.PI),
                new HeadingControlledWaypoint(18, 38, 12, Math.PI, Subroutines.SET_FOUNDATION_LATCHES_OUT),
                new HeadingControlledWaypoint(GRAB_FOUNDATION_LOCATION.x, 38, 8, Math.PI * 0.5),
                GRAB_FOUNDATION_LOCATION,
                new HeadingControlledWaypoint(GRAB_FOUNDATION_LOCATION.x, 40, 6, Math.PI * 0.5, new IgnorantPointTurn(Math.PI, Math.toRadians(10))),
                new Waypoint(36, 36, 6),
                new HeadingControlledWaypoint(-20 + skystoneOffset, 36, 6, Math.PI, Subroutines.ENABLE_INTAKE),

                new HeadingControlledWaypoint(-28 + skystoneOffset, 36, 6, Math.toRadians(225)),
                new StopWaypoint(-36 + skystoneOffset, 28, 4, Math.toRadians(225),
                        3, new JoltsUntilBlockGrab(joltDirection)),
                new HeadingControlledWaypoint(FRONT_PLUNGE_TARGET_X + skystoneOffset, 36, 12, -Math.PI, Subroutines.GRAB_INTAKED_BLOCK),
                // Now make our move to deposit
                new HeadingControlledWaypoint(0, 36, 16, Math.PI),
                new StopWaypoint(DEPOSIT_LOCATION.x, DEPOSIT_LOCATION.y, 8,
                        Math.PI, 8, new DepositUntilSuccessful(ALLIANCE))
        );

        /* Now, we will "charge" along two straight lines until we encounter a block. Where those
        lines are depends on what blocks we grabbed first, since if we grabbed SkystoneState.UPPER
        the two skystones closest to the field wall haven't been touched.
         */

        scoreSkystones.addAll(Waypoint.collate(
                new Waypoint(36, 36, 6),
                new StopWaypoint(0, 36, 6, Math.PI, 0)
        ));

        return scoreSkystones;
    }

    @Override
    public void start() {
        super.start();
        robot.setIntakePower(1);
    }
}
