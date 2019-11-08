package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.ActionAndWait;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.JoltsUntilBlockGrab;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.PointTurnWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.RamFoundationBackward;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.RamFoundationBackwardRed;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;

import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware.FIELD_RADIUS;

// Import constants

@Config
public class SSAutoMovingFoundationRed extends PurePursuitAutoRed {

    StopWaypoint DEPOSIT_LOCATION = new StopWaypoint(FIELD_RADIUS- 20, FIELD_RADIUS - 20, 8,
            Math.PI, 8, new ActionAndWait(1000, Subroutines.SMART_DROP_BLOCK));

    Waypoint GRAB_FOUNDATION_LOCATION = new StopWaypoint(FIELD_RADIUS - (34.5/2), 28,
            8, Math.PI * 0.5, 5, new ActionAndWait(1000, Subroutines.SET_FOUNDATION_LATCHES_DOWN));

    public static double PLUNGE_TARGET_Y = 22;
    public static double BACK_PLUNGE_TARGET_X = -FIELD_RADIUS + 13;
    public static double FRONT_PLUNGE_TARGET_X = -FIELD_RADIUS + 34;

    @Override
    public Pose getBlueStartPosition() {
        return DEFAULT_START_POSITION;
    }

    @Override
    public List<Waypoint> getPurePursuitWaypoints() {
        MecanumPowers joltDirection = (ALLIANCE == Alliance.BLUE) ? MecanumUtil.FORWARD_RIGHT : MecanumUtil.FORWARD_LEFT;

        LinkedList<Waypoint> scoreSkystones = Waypoint.collate(
                new Waypoint(DEFAULT_START_POSITION, 4),

                // We want to move in strictly on the y-axis WRT the field to grab block,
                // which means robot must be at 45 degrees
                new HeadingControlledWaypoint(BACK_PLUNGE_TARGET_X + SKYSTONE.index * 8, 48, 4, -0.75 * Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(BACK_PLUNGE_TARGET_X + SKYSTONE.index * 8, PLUNGE_TARGET_Y, 4, -0.75 * Math.PI,
                        3, new JoltsUntilBlockGrab(joltDirection)),
                new HeadingControlledWaypoint(BACK_PLUNGE_TARGET_X + SKYSTONE.index * 8, 36, 12, -Math.PI, Subroutines.GRAB_INTAKED_BLOCK),

                // Now make our move to deposit
                new Waypoint(18, 40, 12, Subroutines.SET_FOUNDATION_LATCHES_OUT),
                new HeadingControlledWaypoint(GRAB_FOUNDATION_LOCATION.x, 40, 8, Math.PI * 0.5, Subroutines.SET_FOUNDATION_LATCHES_OUT),
                GRAB_FOUNDATION_LOCATION,
                new Waypoint(FIELD_RADIUS - 4 - (34.5/2), 52, 16, Subroutines.SMART_DROP_BLOCK_WITH_LATCHES),
                new HeadingControlledWaypoint(20, 36, 6, Math.toRadians(220)),
                new PointTurnWaypoint(20, 36, 6, Math.toRadians(220), Math.toRadians(10), new RamFoundationBackwardRed(Alliance.RED))
        );

        /* Now, we will "charge" along two straight lines until we encounter a block. Where those
        lines are depends on what blocks we grabbed first, since if we grabbed SkystoneState.UPPER
        the two skystones closest to the field wall haven't been touched.
         */

        double chargePathY1 = 22;

        scoreSkystones.addAll(Waypoint.collate(
                /*new Waypoint(0, 36, 16, Subroutines.ENABLE_INTAKE),
                new Waypoint(-36, 36, 16, Subroutines.CHECK_BLOCK_GRAB),
                new Waypoint(-36, chargePathY1, 16, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(-FIELD_RADIUS + 9, chargePathY1, 4, -Math.PI, 3),
                new Waypoint(0, 36, 16, Subroutines.GRAB_INTAKED_BLOCK),
                new StopWaypoint(DEPOSIT_LOCATION.x, DEPOSIT_LOCATION.y, 8,
                        Math.PI, 8, new ActionAndWait(1000, Subroutines.SMART_DROP_BLOCK)),*/
                /* Park */
                new HeadingControlledWaypoint(24, 40, 6, Math.PI),
                new StopWaypoint(4, 40, 6, Math.PI, 0)
                ));

        return scoreSkystones;
    }

    @Override
    public void start() {
        super.start();
        robot.setIntakePower(1);
    }
}
