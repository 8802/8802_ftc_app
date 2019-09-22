package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.openftc.revextensions2.RevBulkData;

public class SquareAuto extends SimulatableMecanumOpMode {
    MecanumHardware robot;
    PurePursuitPath followPath;

    @Override
    public void init() {
        this.robot = this.getRobot();
        robot.initBNO055IMU(hardwareMap);
        followPath = new PurePursuitPath(robot,
                new Waypoint(0, 0, 8),
                new Waypoint(60, 60, 8),
                new Waypoint(-60, 60, 8),
                new Waypoint(-60, -60, 8),
                new Waypoint(60, -60, 8),
                new Waypoint(0, 0, 8)
        );
    }

    @Override
    public void start() {
        robot.initBulkReadTelemetry();
    }

    @Override
    public void loop() {
        RevBulkData data = robot.performBulkRead();
        followPath.update();
    }

}
