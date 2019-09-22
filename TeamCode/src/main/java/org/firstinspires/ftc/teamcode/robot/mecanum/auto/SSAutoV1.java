package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.HeadingControlledWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.math.Point;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.openftc.revextensions2.RevBulkData;

public class SSAutoV1 extends SimulatableMecanumOpMode {
    public static double FIELD_RADIUS = 141 / 2.0;

    MecanumHardware robot;
    PurePursuitPath followPath;

    public Pose getStartingPosition() {
        return new Pose(-50, FIELD_RADIUS + 9, Math.PI);
    }

    @Override
    public void init() {
        this.robot = this.getRobot();
        robot.initBNO055IMU(hardwareMap);
        followPath = new PurePursuitPath(robot,
                new Waypoint(getStartingPosition(), 4),
                new StopWaypoint(8 * 5 + 9 + 4 - FIELD_RADIUS / 2, 22, 4, Math.PI, 2),
                new StopWaypoint(8 * 5 + 9 - 1 - FIELD_RADIUS / 2, 22, 4, Math.PI, 2),
                new Waypoint(8 * 5 + 9 - 1 - FIELD_RADIUS / 2, 48, 16),
                new Waypoint(48, 48, 16),
                new StopWaypoint(48, 24, 4, -Math.PI/2, 2)
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
