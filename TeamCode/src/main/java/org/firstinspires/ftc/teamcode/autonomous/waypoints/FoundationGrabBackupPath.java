package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class FoundationGrabBackupPath implements Subroutines.ArrivalInterruptSubroutine {
    public static double BACKUP_MS = 250;

    ElapsedTime startTime;

    public FoundationGrabBackupPath() {
        startTime = null;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (startTime == null) {
            startTime = new ElapsedTime();
            robot.setPowers(new MecanumPowers(-0.3, -0.3, -0.3, -0.3));
        } else if (startTime.milliseconds() > BACKUP_MS) {
            Subroutines.GRAB_INTAKED_BLOCK_WITH_LATCHES.runOnce(robot);
            return true;
        }
        return false;
    }
}
