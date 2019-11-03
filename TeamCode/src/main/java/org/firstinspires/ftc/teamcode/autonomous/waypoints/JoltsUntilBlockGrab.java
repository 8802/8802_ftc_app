package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumPowers;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumUtil;

@Config
public class JoltsUntilBlockGrab implements Subroutines.ArrivalInterruptSubroutine {
    public static double JOLT_MS = 500;
    public static double PAUSE_MS = 300;
    public static double MAX_JOLTS = 1;

    int currentJolt;
    long joltStartTimeMS;
    MecanumPowers direction;

    public JoltsUntilBlockGrab(MecanumPowers direction) {
        currentJolt = -1;
        joltStartTimeMS = -1;
        this.direction = direction;
    }

    @Override
    public boolean runCycle(SkystoneHardware robot) {
        if (robot.intakeCurrentQueue.hasBlock()) {
            robot.setIntakePower(0);
            return true; // Advance to next motion path
        }
        long currentTime = System.currentTimeMillis();

        if (currentJolt < 0 || currentTime - joltStartTimeMS > JOLT_MS + PAUSE_MS) {
            // We subtract 1 from MAX_JOLTS because currentJolt starts at -1, and so it ends up at 2
            if (currentJolt >= MAX_JOLTS - 1) {
                robot.setIntakePower(0);
                // If we've failed MAX_JOLTS times, just give up
                return true;
            } else {
                // Otherwise, start a new jolt
                joltStartTimeMS = currentTime;
                currentJolt++;
                robot.setPowers(direction);
            }
        } else if (currentTime - joltStartTimeMS > JOLT_MS) {
            robot.setPowers(MecanumUtil.STOP);
        }
        return false;
    }
}
