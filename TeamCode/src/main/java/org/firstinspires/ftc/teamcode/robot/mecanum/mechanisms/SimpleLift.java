package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.*;

@Config
public class SimpleLift {
    public final int MAX_LAYER = 8;
    public final RevBlinkinLedDriver.BlinkinPattern[] LAYER_PATTERNS = {
            BLACK, RED, GOLD, GREEN, BLUE, VIOLET, WHITE, STROBE_RED, STROBE_WHITE
    };

    // We separate these out to make changing them with FTCDashboard easier
    public static int LAYER_0 = 0;
    public static int LAYER_SHIFT = 500;


    private DcMotorEx lift;
    private RevBlinkinLedDriver leds;
    public int layer;
    public int targetPosition;

    // Also initializes the DcMotor
    public SimpleLift(DcMotorEx lift, RevBlinkinLedDriver leds) {
        this.lift = lift;
        this.leds = leds;
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

    public void setLayer(int layer) {
        this.layer = layer;
        setLiftPositionFromLayer();
    }

    void setLiftPositionFromLayer() {
        // We need to create a new list each tick in case we change things in FTC Dashboard
        targetPosition = LAYER_0 + layer * LAYER_SHIFT;
        lift.setTargetPosition(targetPosition);
        leds.setPattern(LAYER_PATTERNS[layer]);
    }

    public void changePosition(int delta) {
        targetPosition += delta;
        lift.setTargetPosition(targetPosition);
    }

    public void goToMin() {
        lift.setTargetPosition(0);
    }

    public void resetStacking() {
        layer = 0;
        setLiftPositionFromLayer();
    }
}
