package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.*;

@Config
public class SimpleLift {
    public static int MAX_LAYER = 9;
    // We separate these out to make changing them with FTCDashboard easier
    public static int LAYER_0 = 4500;
    public static int DROP_LAYER_0 = 0;
    public static int GRABBING = 0;
    public static int LAYER_SHIFT = 6900;
    public static int UPPER_LAYERS_SHIFT = 0;
    public static int UPPER_LAYERS_START = 5;


    public DoubleMotorLift lift;
    public int layer;
    public int targetPosition;

    // Also initializes the DcMotor
    public SimpleLift(DoubleMotorLift lift) {
        this.lift = lift;
        this.layer = 0;
        this.targetPosition = LAYER_0;
    }

    public void changeLayer(int addend) {
        if (addend + layer >= 0 || addend + layer <= MAX_LAYER) {
            layer += addend;
        }
        setLiftPositionFromLayer();
    }

    public void setLayer(int layer) {
        this.layer = layer;
        setLiftPositionFromLayer();
    }

    public void setLiftPositionFromLayer() {
        targetPosition = LAYER_0 + layer * LAYER_SHIFT;
        if (layer >= UPPER_LAYERS_START) {
            targetPosition += UPPER_LAYERS_SHIFT;
        }
        lift.setTarget(targetPosition);
    }

    public void setLiftPositionWithoutRaise() {
        targetPosition = DROP_LAYER_0 + layer * LAYER_SHIFT;
        if (layer >= UPPER_LAYERS_START) {
            targetPosition += UPPER_LAYERS_SHIFT;
        }
        lift.setTarget(targetPosition);
    }

    public void changePosition(int delta) {
        targetPosition += delta;
        lift.setTarget(targetPosition);
    }

    public void cacheToGrabbing() {
        lift.setTarget(GRABBING);
    }

    public boolean up() {
        return lift.getTarget() > GRABBING;
    }
}
