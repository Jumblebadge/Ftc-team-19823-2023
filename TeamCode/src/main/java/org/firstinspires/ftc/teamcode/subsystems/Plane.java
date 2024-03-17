package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.ServoImplExW;

public class Plane {

    private final ServoImplExW plane;
    public final double HOLD = 0.5, SHOOT = 0.5;

    public Plane(HardwareMap hardwareMap){
        plane = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "plane"));
        plane.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public void hold(){ plane.setPosition(HOLD); }

    public void shoot(){ plane.setPosition(SHOOT); }

    public void setPosition(double position) { plane.setPosition(position); }

    public void moveTo(double target) {
        plane.setPosition(target);
    }

    public void PWMrelease() { plane.setPwmDisable(); }

    public void toggle(boolean active) {
        if (active) { shoot(); }
        else { hold(); }
    }

}
