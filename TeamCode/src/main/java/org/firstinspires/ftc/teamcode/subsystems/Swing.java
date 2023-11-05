package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.TwoServo;

public class Swing {

    public static final double score = 0.5, init = 0.5, transfer = 0.5;
    private final TwoServo swing;

    public Swing(HardwareMap hardwareMap) {
        ServoImplEx swingL = hardwareMap.get(ServoImplEx.class, "swingL");
        ServoImplEx swingR = hardwareMap.get(ServoImplEx.class, "swingR");

        swingL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        swingR.setPwmRange(new PwmControl.PwmRange(500, 2500));

        swing = new TwoServo(swingL, swingR);
    }

    public void moveTo(double target) {
        swing.setPosition(target);
    }

    public void PWMrelease() {
        swing.PWMrelease();
    }

    public void score() {
        swing.setPosition(score);
    }

    public void transfer() {
        swing.setPosition(transfer);
    }

    public void init() {
        swing.setPosition(init);
    }

    public void toggle(boolean active) {
        if (active) { score(); }
        else { transfer(); }
    }


}
