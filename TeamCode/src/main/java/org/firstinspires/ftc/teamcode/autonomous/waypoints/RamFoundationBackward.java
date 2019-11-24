package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.elements.Alliance;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class RamFoundationBackward implements Subroutines.ArrivalInterruptSubroutine {

    ElapsedTime timer;

    public RamFoundationBackward() {
        this.timer = null;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (this.timer == null) {
            robot.setPowers(new MecanumPowers(-0.5, -0.5, -0.5, -0.5));
            this.timer = new ElapsedTime();
        }

        // If the robot is stopped or we've been here for two seconds
        if (robot.localizer.relVelocity().radius() < 1 ||
                timer.milliseconds() > 2000) {
            robot.leftFoundationLatch.retract();
            robot.rightFoundationLatch.retract();
            return true;
        }
        return false;
    }
}
