package org.firstinspires.ftc.teamcode.robot.mecanum.mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class SimpleLift {
    /* Our robot uses a two-motor lift, and it's really important the two motors be kept in sync.
    We tried using PID from the phone to accomplish this, but results were poor. Instead, we split
    our single external encoder cable to go into both motor ports,
     */
    public static int MAX_LAYER = 13;

    // The position we should go to for grabbing blocks
    public static int GRABBING = 0;

    // LAYER_SHIFT is the distance in ticks between each two layers
    public static int LAYER_SHIFT = 100;

    // PLACEMENT_LAYER_0 is the position at which we should place blocks on layer 0
    public static int PLACEMENT_LAYER_0 = 0;

    // VERIFY_OFFSET is how far upwards we should go for maneuvering blocks
    public static int VERIFY_OFFSET = 50;

    /* If we're more than PID_RANGE ticks BELOW our target, we'll just set power to 100% */
    public static int PID_RANGE = 25;

    public int layer;
    public int targetPosition;
    private boolean pidControlled;

    public DcMotorEx left, right;

    // Also initializes the DcMotor
    public SimpleLift(DcMotorEx left, DcMotorEx right) {
        this.left = left;
        this.right = right;
        left.setTargetPosition(GRABBING);
        right.setTargetPosition(GRABBING);
        doPID();
        left.setPower(1);
        right.setPower(1);
    }

    private void doPID() {
        if (!pidControlled) {
            left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left.setPower(1);
            right.setPower(1);
            pidControlled = true;
        }
    }

    private void doPower() {
        if (pidControlled) {
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left.setPower(1);
            right.setPower(1);
            pidControlled = false;
        }
    }

    // This function is only called by SkystoneTeleop, where it's used to adjust the position
    // prior to placing a block down. Thus, we want to maneuver into the raised position
    public void changeLayer(int addend) {
        if (addend + layer >= 0 || addend + layer <= MAX_LAYER) {
            layer += addend;
        }
        setRaisedPositionFromLayer();
    }

    // This function is called by our auto code, which doesn't really care about stacking.
    // So, we just go right to our target position
    public void setLayer(int layer) {
        this.layer = layer;
        setDroppedPositionFromLayer();
    }

    private void setPosition(int position) {
        left.setTargetPosition(position);
        right.setTargetPosition(position);
    }

    private void updatePosition() {
        setPosition(targetPosition);
    }

    public void setRaisedPositionFromLayer() {
        targetPosition = PLACEMENT_LAYER_0 + VERIFY_OFFSET + layer * LAYER_SHIFT;
        updatePosition();
    }

    public void setDroppedPositionFromLayer() {
        targetPosition = PLACEMENT_LAYER_0 + layer * LAYER_SHIFT;
        updatePosition();
    }

    public void changePosition(int delta) {
        targetPosition += delta;
        updatePosition();
    }

    public void cacheToGrabbing() {
        targetPosition = GRABBING;
        updatePosition();
    }

    // Depending on how far away we are from the position, it might be faster to use PID to get,
    // or we might just YEET up by setting both motors to max power. We pick which one depending
    // on how far away we are from the target.
    public void update() {
        // Don't spend time querying motor position if the lift isn't raised
        if (targetPosition > PID_RANGE) {
            int position = left.getCurrentPosition();
            if (left.getCurrentPosition() + PID_RANGE < targetPosition) {
                doPower();
            } else {
                doPID();
            }
        } else {
            doPID();
        }
    }

    public boolean up() {
        return targetPosition > GRABBING;
    }
}
