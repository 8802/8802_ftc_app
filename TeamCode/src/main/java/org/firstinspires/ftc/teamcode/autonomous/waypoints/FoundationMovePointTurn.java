package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.controllers.MecanumPurePursuitController;
import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class IgnorantPointTurn implements Subroutines.ArrivalInterruptSubroutine {
    public static double REDUCTION_DIST = Math.PI/3;

    double targetHeading;
    double allowedError;

    public IgnorantPointTurn(double targetHeading, double allowedError) {
        this.targetHeading = targetHeading;
        this.allowedError = allowedError;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        double currentHeading = robot.pose().heading;

        double angleToTarget = MathUtil.angleWrap(targetHeading - currentHeading);
        Pose poseTurnPower = new Pose(0, 0, angleToTarget / REDUCTION_DIST);
        robot.setPowers(new MecanumPowers(poseTurnPower));

        if (Math.abs(currentHeading - targetHeading) < allowedError) {
            robot.leftFoundationLatch.retract();
            robot.rightFoundationLatch.retract();
            return true;
        }
        return false;
    }
}
