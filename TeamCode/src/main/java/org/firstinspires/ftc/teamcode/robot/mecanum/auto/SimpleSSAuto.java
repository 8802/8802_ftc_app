package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware.FIELD_RADIUS;

// Import constants

@Autonomous
@Config
public class SimpleSSAuto extends SimulatableMecanumOpMode {

    Waypoint DEPOSIT_LOCATION = new StopWaypoint(24, 30, 4, 0.75 * Math.PI, 3);
    Pose START_POSITION = new Pose(-FIELD_RADIUS + 22.75 + 9, FIELD_RADIUS - 9, 3 * Math.PI / 2);

    public static double PLUNGE_TARGET_Y = 22;
    public static double BACK_PLUNGE_TARGET_X = -FIELD_RADIUS + 13;
    public static double FRONT_PLUNGE_TARGET_X = -FIELD_RADIUS + 34;


    MecanumHardware robot;
    PurePursuitPath followPath;
    FtcDashboard dashboard;

    //public static SkystoneState SKYSTONE = SkystoneState.MIDDLE;
    public static int SKYSTONE = 1;

    @Override
    public void init() {
        this.dashboard = FtcDashboard.getInstance();
        this.robot = this.getRobot(START_POSITION);
        robot.initBNO055IMU(hardwareMap);
        followPath = new PurePursuitPath(robot,
                new Waypoint(START_POSITION, 4),

                // We want to move in strictly on the y-axis WRT the field to grab block,
                // which means robot must be at 45 degrees
                new HeadingControlledWaypoint(BACK_PLUNGE_TARGET_X + SKYSTONE * 8, 48, 4, -0.75 * Math.PI, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(BACK_PLUNGE_TARGET_X + SKYSTONE * 8, PLUNGE_TARGET_Y, 4, -0.75 * Math.PI,
                        3, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(BACK_PLUNGE_TARGET_X - 8 + SKYSTONE * 8, PLUNGE_TARGET_Y, 4, -0.75 * Math.PI,
                        3, Subroutines.CHECK_BLOCK_GRAB),
                new HeadingControlledWaypoint(BACK_PLUNGE_TARGET_X + SKYSTONE * 8, 48, 12, -Math.PI),
                // Now make our move to deposit
                new Waypoint(0, 48, 16),
                DEPOSIT_LOCATION,

                new Waypoint(FRONT_PLUNGE_TARGET_X + SKYSTONE * 8, 48, 8, Subroutines.ENABLE_INTAKE),
                new StopWaypoint(FRONT_PLUNGE_TARGET_X + SKYSTONE * 8, PLUNGE_TARGET_Y, 4, -0.75 * Math.PI,
                        3, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(FRONT_PLUNGE_TARGET_X - 8 + SKYSTONE * 8, PLUNGE_TARGET_Y, 4, -0.75 * Math.PI,
                        3, Subroutines.CHECK_BLOCK_GRAB),
                new HeadingControlledWaypoint(FRONT_PLUNGE_TARGET_X + SKYSTONE * 8, 48, 12, -Math.PI),
                // Now make our move to deposit
                new Waypoint(0, 48, 16),
                DEPOSIT_LOCATION,

                new Waypoint(0, 36, 16, Subroutines.ENABLE_INTAKE),
                new Waypoint(-24, 36, 16, Subroutines.CHECK_BLOCK_GRAB),
                new Waypoint(-24, 18, 16, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(-FIELD_RADIUS + 9, 18, 4, -Math.PI, 3),
                new Waypoint(0, 48, 16),
                DEPOSIT_LOCATION,

                new Waypoint(0, 36, 16, Subroutines.ENABLE_INTAKE),
                new Waypoint(-24, 36, 16, Subroutines.CHECK_BLOCK_GRAB),
                new Waypoint(-24, 12, 16, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(-FIELD_RADIUS + 9, 12, 4, -Math.PI, 3),
                new Waypoint(0, 48, 16),
                DEPOSIT_LOCATION
        );
    }

    @Override
    public void start() {
        robot.initBulkReadTelemetry();
        robot.setIntakePower(1);
    }

    @Override
    public void loop() {
        RevBulkData data = robot.performBulkRead();
        robot.drawDashboardPath(followPath);
        robot.sendDashboardTelemetryPacket();

        if (!followPath.finished()) {
            followPath.update();
        } else {
            robot.setPowers(MecanumUtil.STOP);
        }
    }
}
