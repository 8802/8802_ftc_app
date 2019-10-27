package org.firstinspires.ftc.teamcode.autonomous.waypoints;

import org.firstinspires.ftc.teamcode.common.math.Pose;

public class LocalizationErrorAdjustSubroutine implements Subroutines.Subroutine {
    public Pose adjustment;

    public LocalizationErrorAdjustSubroutine(Pose adjustment) {
        this.adjustment = adjustment;
    }
}
