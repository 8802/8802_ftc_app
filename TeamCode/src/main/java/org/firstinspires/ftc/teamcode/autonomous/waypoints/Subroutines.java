package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumHardware;
import org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms.IntakeCurrents;

public class Subroutines {
    public interface Subroutine {}

    public interface OnceOffSubroutine extends Subroutine {
        void runOnce(MecanumHardware robot);
    }

    public interface RepeatedSubroutine extends Subroutine {
        boolean runLoop(MecanumHardware robot); // Returns whether we should advance
    }

    public static final OnceOffSubroutine ENABLE_INTAKE = (robot) -> { robot.setIntakePower(1); };
    public static final OnceOffSubroutine STOP_INTAKE = (robot) -> { robot.setIntakePower(0); };
    public static final OnceOffSubroutine REVERSE_INTAKE = (robot) -> { robot.setIntakePower(-1); };

    public static final RepeatedSubroutine CHECK_BLOCK_GRAB = (robot) -> {
        IntakeCurrents powerDraw = robot.getIntakeCurrent();
        if (powerDraw.hasBlock()) {
            robot.setIntakePower(0); // Disable intake
            return true; // Advance to next motion path
        }
        return false;
    };


}