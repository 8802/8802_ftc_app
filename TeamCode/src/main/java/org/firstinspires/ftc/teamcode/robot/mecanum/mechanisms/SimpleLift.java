package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class SimpleLift {
    public final int MAX_LAYER = 4;

    // We separate these out to make changing them with FTCDashboard easier
    public static int LAYER_0 = 0;
    public static int LAYER_1 = 1000;
    public static int LAYER_2 = 2000;
    public static int LAYER_3 = 3000;
    public static int LAYER_4 = 4000;


    private DcMotorEx lift;
    public int layer;
    public int targetPosition;

    // Also initializes the DcMotor
    public SimpleLift(DcMotorEx lift) {
        this.lift = lift;
        lift.setTargetPosition(LAYER_0);
        lift.setPower(1);
        lift.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

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
        int[] layers = new int[] {LAYER_0, LAYER_1, LAYER_2, LAYER_3, LAYER_4};
        lift.setTargetPosition(layers[layer]);
        targetPosition = layers[layer];
    }

    public void changePosition(int delta) {
        targetPosition = Math.max(LAYER_0, Math.min(LAYER_4, targetPosition + delta));
        lift.setTargetPosition(targetPosition);
    }
}
