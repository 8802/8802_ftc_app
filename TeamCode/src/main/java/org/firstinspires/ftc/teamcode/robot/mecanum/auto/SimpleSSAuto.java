package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Subroutines;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.elements.SkystoneState;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.openftc.revextensions2.RevBulkData;

import static org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware.FIELD_RADIUS;

// Import constants

@Autonomous
public class SimpleSSAuto extends SimulatableMecanumOpMode {

    Waypoint DEPOSIT_LOCATION = new StopWaypoint(24, 30, 4, 0.75 * Math.PI, 3);
    double GRAB_HEADING = -0.85 * Math.PI;
    Pose START_POSITION = new Pose(-FIELD_RADIUS + 22.75 + 9, FIELD_RADIUS - 9, 3 * Math.PI / 2);

    MecanumHardware robot;
    PurePursuitPath followPath;
    FtcDashboard dashboard;

    public static SkystoneState SKYSTONE = SkystoneState.MIDDLE;

    @Override
    public void init() {
        this.dashboard = FtcDashboard.getInstance();
        this.robot = this.getRobot(START_POSITION);
        robot.initBNO055IMU(hardwareMap);
        followPath = new PurePursuitPath(robot,
                new Waypoint(START_POSITION, 4, Subroutines.CHECK_BLOCK_GRAB),

                // Grab the farthest block and move foundation into position
                new StopWaypoint(-FIELD_RADIUS + 6 + 4, 24, 4, -0.75 * Math.PI, 3, Subroutines.STOP_INTAKE),
                new Waypoint(-FIELD_RADIUS + 4 + 4, 36, 16),
                new Waypoint(0, 36, 16),
                DEPOSIT_LOCATION,

                new Waypoint(0, 36, 16, Subroutines.ENABLE_INTAKE),
                new Waypoint(-24, 36, 16, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(-FIELD_RADIUS + 28 + 4, 24, 4, -0.75 * Math.PI, 3, Subroutines.STOP_INTAKE),
                new Waypoint(-FIELD_RADIUS + 28 + 4, 48, 16),
                new Waypoint(0, 48, 16),
                DEPOSIT_LOCATION,

                new Waypoint(0, 36, 16, Subroutines.ENABLE_INTAKE),
                new Waypoint(-24, 36, 16, Subroutines.CHECK_BLOCK_GRAB),
                new StopWaypoint(-48, 24, 4, -0.75 * Math.PI, 3, Subroutines.STOP_INTAKE),
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
