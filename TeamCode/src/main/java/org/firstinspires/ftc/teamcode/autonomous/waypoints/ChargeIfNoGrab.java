package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class ChargeIfNoGrab implements Subroutines.ArrivalInterruptSubroutine {
    public static double WAIT_MS = 500;
    public static double CHARGE_MS = 500;

    ElapsedTime timer;

    public ChargeIfNoGrab() {
        timer = null;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (robot.hasBlockInClaws() || robot.hasBlockInTray()) {
            return true; // Advance to next motion path
        }

        if (timer == null) {
            timer = new ElapsedTime();
            robot.setPowers(MecanumUtil.STOP);
        }

        if (timer.milliseconds() > WAIT_MS + CHARGE_MS) {
            return true;
        } else if (timer.milliseconds() > WAIT_MS) {
            robot.setPowers(new MecanumPowers(0.5, 0, 0));
        }
        return false;
    }
}
