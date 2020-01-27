package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

public class SkipFifthBlockPlacement implements Subroutines.MetaSubroutine {
    static ElapsedTime timeSinceStart = new ElapsedTime();

    public static double NO_PLACEMENT_DEADLINE = 27; // Seconds


    public SkipFifthBlockPlacement() {
        timeSinceStart.reset();
    }

    @Override
    public void runOnce(PurePursuitPath path, SkystoneHardware robot) {
        // If we don't have a block or we've passed the no placement deadline, skip to the end
        if (!robot.hasBlockInTray() || timeSinceStart.seconds() > NO_PLACEMENT_DEADLINE) {
            path.currPoint = path.waypoints.size() - 2;
        }
    }
}
