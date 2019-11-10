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

    StopWaypoint DEPOSIT_LOCATION = new StopWaypoint(FIELD_RADIUS - 30, FIELD_RADIUS - 24, 8,
            Math.PI, 8, new ActionAndWait(1000, Subroutines.SMART_DROP_BLOCK));

    Waypoint GRAB_FOUNDATION_LOCATION = new StopWaypoint(FIELD_RADIUS - (34.5/2), 28,
            8, Math.PI * 0.5, 6, new ActionAndWait(1000, Subroutines.GRAB_INTAKED_BLOCK_WITH_LATCHES));

    public static double PLUNGE_TARGET_Y = 22;
    public static double BACK_PLUNGE_TARGET_X = -FIELD_RADIUS + 13;
    public static double FRONT_PLUNGE_TARGET_X = -FIELD_RADIUS + 37;

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
                new HeadingControlledWaypoint(BACK_PLUNGE_TARGET_X + SKYSTONE.index * 8, 36, 8, -Math.PI, Subroutines.OPTIONALLY_REJECT_BLOCK),

                // Now make our move to deposit
                new HeadingControlledWaypoint(18, 38, 12, Math.PI, Subroutines.SET_FOUNDATION_LATCHES_OUT),
                new HeadingControlledWaypoint(GRAB_FOUNDATION_LOCATION.x, 40, 8, Math.PI * 0.5),
                GRAB_FOUNDATION_LOCATION,
                new Waypoint(FIELD_RADIUS - 4 - (34.5/2), 52, 16),
                new HeadingControlledWaypoint(20, 36, 6, Math.toRadians(220), Subroutines.SMART_DROP_BLOCK_WITH_LATCHES),
                new PointTurnWaypoint(20, 36, 6, Math.toRadians(230), Math.toRadians(10), new RamFoundationBackwardRed(Alliance.RED)),

                new HeadingControlledWaypoint(36, 40, 6, Math.PI),
                new HeadingControlledWaypoint(0, 40, 6, Math.PI),

                new Waypoint(FRONT_PLUNGE_TARGET_X + SKYSTONE.index * 8, 48, 6, Subroutines.ENABLE_INTAKE),
                new StopWaypoint(FRONT_PLUNGE_TARGET_X + SKYSTONE.index * 8, PLUNGE_TARGET_Y, 4, -0.75 * Math.PI,
                        3, new JoltsUntilBlockGrab(joltDirection)),
                new HeadingControlledWaypoint(FRONT_PLUNGE_TARGET_X + SKYSTONE.index * 8, 36, 8, -Math.PI, Subroutines.GRAB_INTAKED_BLOCK),
                // Now make our move to deposit
                new HeadingControlledWaypoint(0, 36, 16, Math.PI, Subroutines.OPTIONALLY_REJECT_BLOCK),
                new StopWaypoint(DEPOSIT_LOCATION.x, DEPOSIT_LOCATION.y, 8,
                        Math.PI, 8, new ActionAndWait(1000, Subroutines.SMART_DROP_BLOCK))
        );

        /* Now, we will "charge" along two straight lines until we encounter a block. Where those
        lines are depends on what blocks we grabbed first, since if we grabbed SkystoneState.UPPER
        the two skystones closest to the field wall haven't been touched.
         */

        double chargePathY1 = 22;

        if (SKYSTONE == SkystoneState.UPPER) {
            scoreSkystones.addAll(Waypoint.collate(
                    new Waypoint(36, 36, 6, Subroutines.SKIP_TO_END_IF_BAD_STATE),
                    new HeadingControlledWaypoint(0, 36, 6, Math.PI, Subroutines.ENABLE_INTAKE),
                    new Waypoint(-36, 36, 16, Subroutines.CHECK_BLOCK_GRAB),
                    new Waypoint(-36, chargePathY1, 16, Subroutines.CHECK_BLOCK_GRAB),
                    new StopWaypoint(-FIELD_RADIUS + 22, chargePathY1, 3, -Math.PI, 4),
                    new Waypoint(-FIELD_RADIUS + 22, 40, 6, Subroutines.OPTIONALLY_REJECT_BLOCK)
            ));
        } else if (SKYSTONE == SkystoneState.MIDDLE) {
            scoreSkystones.addAll(Waypoint.collate(
                    new Waypoint(36, 36, 6, Subroutines.SKIP_TO_END_IF_BAD_STATE),
                    new HeadingControlledWaypoint(0, 36, 6, Math.PI, Subroutines.ENABLE_INTAKE),
                    new Waypoint(-36, 36, 16, Subroutines.CHECK_BLOCK_GRAB),
                    new Waypoint(-36, chargePathY1, 16, Subroutines.CHECK_BLOCK_GRAB),
                    new StopWaypoint(-FIELD_RADIUS + 14, chargePathY1, 4, -Math.PI, 4),
                    new Waypoint(-FIELD_RADIUS + 22, 40, 6, Subroutines.OPTIONALLY_REJECT_BLOCK)
            ));
        }

        if (SKYSTONE == SkystoneState.UPPER || SKYSTONE == SkystoneState.MIDDLE) {
            //Return and deposit are all same
            scoreSkystones.addAll(Waypoint.collate(
                    new HeadingControlledWaypoint(0, 38, 8, Math.PI, Subroutines.GRAB_INTAKED_BLOCK),
                    new StopWaypoint(DEPOSIT_LOCATION.x, DEPOSIT_LOCATION.y, 8,
                            Math.PI, 8, new ActionAndWait(1000, Subroutines.SMART_DROP_BLOCK)),
                    new HeadingControlledWaypoint(24, 34, 6, Math.PI, Subroutines.STOP_INTAKE),
                    new StopWaypoint(4, 34, 6, Math.PI, 0)
            )       );
        }

        scoreSkystones.addAll(Waypoint.collate(
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
