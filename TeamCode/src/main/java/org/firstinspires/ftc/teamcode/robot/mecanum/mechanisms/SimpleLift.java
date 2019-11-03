package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class SimpleLift {
    public final int MAX_LAYER = 4;

    // We separate these out to make changing them with FTCDashboard easier
    public static int LAYER_0 = 0;
    public static int LAYER_SHIFT = 500;
    public static int MAX_POSITION = 3000;


    private DcMotorEx lift;
    public int layer;
    public int targetPosition;

    // Also initializes the DcMotor
    public SimpleLift(DcMotorEx lift) {
        this.lift = lift;
        lift.setPower(0); // Set power to zero before switching modes to stop jumping
        lift.setTargetPosition(LAYER_0);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.setPower(1);
        this.layer = 0;
        this.targetPosition = LAYER_0;
    }

    public void changeLayer(int addend) {
        if (addend + layer < 0 || addend + layer > MAX_LAYER) {
            return; // Don't change anything if we would go out of bounds
        }
        layer += addend;
        setLiftPositionFromLayer();
    }

    private void setLiftPositionFromLayer() {
        // We need to create a new list each tick in case we change things in FTC Dashboard
        targetPosition = LAYER_0 + layer * LAYER_SHIFT;
        lift.setTargetPosition(targetPosition);
    }

    public void changePosition(int delta) {
        targetPosition += delta;
        lift.setTargetPosition(targetPosition);
    }

    public void goToMin() {
        layer = 0;
        setLiftPositionFromLayer();
    }
}
