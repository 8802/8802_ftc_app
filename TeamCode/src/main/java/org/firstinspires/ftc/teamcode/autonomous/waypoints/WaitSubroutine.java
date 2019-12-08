package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

public class WaitSubroutine implements Subroutines.ArrivalInterruptSubroutine {
    double waitMS;
    ElapsedTime timer;

    public WaitSubroutine(double waitMS) {
        this.waitMS = waitMS;
        this.timer = null;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (timer == null) {
            timer = new ElapsedTime();
        }

        return timer.milliseconds() > waitMS;
    }
}
