package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.StopWaypoint;
import org.firstinspires.ftc.teamcode.autonomous.waypoints.Waypoint;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.common.elements.SkystoneState;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.openftc.revextensions2.RevBulkData;

import java.util.LinkedList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware.FIELD_RADIUS;

@Autonomous(name="Test Pure Pursuit", group="Testing")
public class TestPurePursuitAuto extends SimulatableMecanumOpMode {
    Pose DEFAULT_START_POSITION = new Pose(-FIELD_RADIUS + 22.75 + 9, FIELD_RADIUS - 9, 3 * Math.PI / 2);

    SkystoneHardware robot;
    PurePursuitPath followPath;

    // Robot state

    public List<Waypoint> getPurePursuitWaypoints() {
        return  Waypoint.collate(
                new Waypoint(DEFAULT_START_POSITION, 4),
                new StopWaypoint(DEFAULT_START_POSITION.x, -38.5, 20, 1.5 * Math.PI, 0)
        );
    }

    @Override
    public void init() {
        Pose start = DEFAULT_START_POSITION.clone();
        this.robot = this.getRobot(start);
    }

    @Override
    public void start() {
        telemetry.clearAll();
        robot.initBulkReadTelemetry();
        followPath = new PurePursuitPath(robot, getPurePursuitWaypoints());
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
            stop();
        }
    }

    @Override
    public void stop() {
        robot.blockGrabber.retract();
    }
}
