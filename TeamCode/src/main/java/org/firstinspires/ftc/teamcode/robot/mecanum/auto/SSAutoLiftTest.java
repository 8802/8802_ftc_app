package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.waypoints.CloseDepositUntilSuccessful;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.DepositUntilSuccessful;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.FirstBlockDepositStop;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.FoundationGrabBackupPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.GrabBlockOptionallyRejectDouble;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.JoltsUntilBlockGrab;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.SkipFifthBlockPlacement;
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
@Autonomous(name="Test skystone auto")
public class SSAutoLiftTest extends PurePursuitAuto {

    Waypoint GRAB_FOUNDATION_LOCATION = new StopWaypoint(FIELD_RADIUS - 10 - (34.5/2), 30,
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

        LinkedList<Waypoint> waypoints = Waypoint.collate(
                new Waypoint(DEFAULT_START_POSITION.x, 16, 4),
                new StopWaypoint(DEFAULT_START_POSITION.x, 0, 12, 1.5 * Math.PI, 0, Subroutines.LIFT_A_LITTLE),
                new Waypoint(DEFAULT_START_POSITION.x, 18, 12),
                new StopWaypoint(DEFAULT_START_POSITION.x, -38.5, 12, 1.5 * Math.PI, 0)
        );


        return waypoints;
    }

    @Override
    public void start() {
        super.start();
        Subroutines.ENABLE_INTAKE.runOnce(robot);
    }
}
