package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.common.math.MathUtil;
import org.firstinspires.ftc.teamcode.common.math.Pose;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class FirstBlockDepositStop implements Subroutines.ArrivalInterruptSubroutine {
    public static double REDUCTION_DIST = Math.PI/3;
    public static double TARGET_HEADING = Math.PI;

    public FirstBlockDepositStop() {}

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        // If we don't have a "no check" action, then verify we have a block
        if (!robot.hasAction("SKYSTONE1DEPOSITNOCHECK") && robot.hasBlockInTray()) {
            robot.actionCache.clear(); // Remove everything from action cache

            // Grab the block
            robot.pidLift.setLayer(0);
            robot.blockGrabber.retract(); // Open claw
            robot.blockFlipper.readyBlockGrab(); // Move flipper
            robot.actionCache.add(new DelayedSubroutine(500, Subroutines.CLOSE_CLAW, "SKYSTONE1DEPOSITNOCHECK"));
            robot.actionCache.add(new DelayedSubroutine(1000, Subroutines.SET_FLIPPER_NORM_EXTEND, "SKYSTONE1DEPOSITNOCHECK"));
            robot.actionCache.add(new DelayedSubroutine(1500, Subroutines.OPEN_CLAW, "SKYSTONE1DEPOSITNOCHECK"));
            robot.actionCache.add(new DelayedSubroutine(1750, Subroutines.LIFT_A_LITTLE, "SKYSTONE1DEPOSIT"));
            robot.actionCache.add(new DelayedSubroutine(2250, Subroutines.SET_FLIPPER_INTAKING, "SKYSTONE1DEPOSIT"));
            robot.actionCache.add(new DelayedSubroutine(2250, Subroutines.LOWER_LIFT_TO_GRABBING, "SKYSTONE1DEPOSITEND"));
        }
        // If we don't have a deposit end action, we're done!
        if (!robot.hasAction("SKYSTONE1DEPOSITEND")) {
            return true;
        } else {
            // Might as well be finishing adjusting our heading while we're here
            double currentHeading = robot.pose().heading;
            double angleToTarget = MathUtil.angleWrap(TARGET_HEADING - currentHeading);
            Pose poseTurnPower = new Pose(0, 0, angleToTarget / REDUCTION_DIST);
            robot.setPowers(new MecanumPowers(poseTurnPower));
            return false;
        }
    }
}
