package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.ServoGroup;
import org.firstinspires.ftc.teamcode.utility.ServoImplExW;

public class Clamp {

    private final ServoGroup clamp;
    public final double score = 0.5, transfer = 0.5, init = 0.5;

    public Clamp(HardwareMap hardwareMap){

        ServoImplExW clampL = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "clampL"));
        ServoImplExW clampR = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "clampR"));

        clampL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clampR.setPwmRange(new PwmControl.PwmRange(500, 2500));

        clamp = new ServoGroup(clampL, clampR);
    }

    public void setPosition(double target) {
        clamp.setPosition(target);
    }

    public void PWMrelease() { clamp.PWMrelease(); }

    public void score() {
        clamp.setPosition(score);
    }

    public void transfer() { clamp.setPosition(transfer); }

    public void init() {
        clamp.setPosition(init);
    }

    public void toggle(boolean active) {
        if (active) { score(); }
        else { transfer(); }
    }

}
