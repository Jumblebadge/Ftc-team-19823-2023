package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Plane {

    private final ServoImplEx plane;
    private final double hold = 0.5, release = 0.5;

    public Plane(HardwareMap hardwareMap){
        plane = hardwareMap.get(ServoImplEx.class, "plane");
        plane.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void hold(){ plane.setPosition(hold); }

    public void release(){ plane.setPosition(release); }

    public void moveTo(double target) {
        plane.setPosition(target);
    }

    public void PWMrelease() { plane.setPwmDisable(); }

    public void toggle(boolean active) {
        if (active) { release(); }
        else { hold(); }
    }

}
