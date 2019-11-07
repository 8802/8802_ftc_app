package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class ActionAndWait implements Subroutines.ArrivalInterruptSubroutine {

    double waitMS;
    Subroutines.OnceOffSubroutine action;
    ElapsedTime timer;

    public ActionAndWait(double waitMS, Subroutines.OnceOffSubroutine action) {
        this.waitMS = waitMS;
        this.action = action;
        this.timer = null;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (this.timer == null) {
            robot.setPowers(MecanumUtil.STOP);
            action.runOnce(robot);
            this.timer = new ElapsedTime();
        }
        return timer.milliseconds() > waitMS;
    }
}
