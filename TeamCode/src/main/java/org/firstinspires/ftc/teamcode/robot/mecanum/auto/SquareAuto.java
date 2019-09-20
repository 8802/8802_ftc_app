package org.firstinspires.ftc.teamcode.robot.mecanum.auto;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.controllers.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.autonomous.controllers.PurePursuitPath;
import org.firstinspires.ftc.teamcode.common.SimulatableMecanumOpMode;
import org.firstinspires.ftc.teamcode.common.math.Point;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.openftc.revextensions2.RevBulkData;

public class SquareAuto extends SimulatableMecanumOpMode {
    MecanumHardware robot;
    PurePursuitPath followPath;

    @Override
    public void init() {
        this.robot = this.getRobot();
        robot.initBNO055IMU(hardwareMap);
        followPath = new PurePursuitPath(robot,
                new Point(0, 0),
                new Point(50, 50),
                new Point(-50, 50),
                new Point(-50, -50),
                new Point(50, -50),
                new Point(0, 0)
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
