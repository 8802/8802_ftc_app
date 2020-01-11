package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

public class OptionalSkipFourthBlock implements Subroutines.MetaSubroutine {

    // The time must be before 24 seconds if we are to go for a fourth block
    public static int THREE_BLOCK_DEADLINE = 24;

    int skips;
    OpMode opMode;

    public OptionalSkipFourthBlock(OpMode opMode, int skips) {
        this.skips = skips;
        this.opMode = opMode;
    }

    @Override
    public void runOnce(PurePursuitPath path, SkystoneHardware robot) {
        if (opMode.getRuntime() > THREE_BLOCK_DEADLINE) {
            path.currPoint += skips;
        }
    }
}
