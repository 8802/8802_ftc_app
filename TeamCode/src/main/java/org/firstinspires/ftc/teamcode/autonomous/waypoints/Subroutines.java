package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.autonomous.PurePursuitPath;
import org.firstinspires.ftc.teamcode.robot.mecanum.SkystoneHardware;

@Config
public class Subroutines {
    public interface Subroutine {}

    public interface OnceOffSubroutine extends Subroutine {
        void runOnce(SkystoneHardware robot);
    }

    public interface MetaSubroutine extends Subroutine {
        void runOnce(PurePursuitPath path);
    }

    public interface RepeatedSubroutine extends Subroutine {
        boolean runLoop(SkystoneHardware robot); // Returns whether we should advance
    }

    // Once a position is reached, if that position has an ArrivalInterruptSubroutine, we advance to
    // the next waypoint (so the waypoint with the ArrivalInterruptSubroutine is our current waypoint)
    // and we call runCycle every tick until it eventually returns true
    public interface ArrivalInterruptSubroutine extends Subroutine {
        boolean runCycle(SkystoneHardware robot); // Returns whether it's complete
    }

    public static final OnceOffSubroutine ENABLE_INTAKE = (robot) -> { robot.setIntakePower(1); };
    public static final OnceOffSubroutine STOP_INTAKE = (robot) -> { robot.setIntakePower(0); };
    public static final OnceOffSubroutine REVERSE_INTAKE = (robot) -> { robot.setIntakePower(-1); };

    public static final RepeatedSubroutine CHECK_BLOCK_GRAB = (robot) -> {
        return robot.intakeCurrentQueue.hasBlock();
    };
}