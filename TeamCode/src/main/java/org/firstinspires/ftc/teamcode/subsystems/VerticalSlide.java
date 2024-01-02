package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.MotorGroup;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

public class VerticalSlide {

    private final MotorGroup motors;
    private double target;
    private final TouchSensor touch;
    private final RunMotionProfile profile = new RunMotionProfile(60000,70000,80000,0.1,0,1,0.2, 1);

    public static final double high = 1700, mid = 400, transfer = 276.333333333, autotransfer = 300, zero = 0;
    private double currentHeight = zero, offset = 0;

    // 0-1600

    public VerticalSlide(HardwareMap hardwareMap){
        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class,"Llift");
        DcMotorEx liftRight = hardwareMap.get(DcMotorEx.class,"Rlift");
        touch = hardwareMap.get(TouchSensor.class, "Vtouch");

        liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new MotorGroup(liftLeft,liftRight);
    }

    public void moveTo(double target){
        this.target = target;
    }

    public void update() {
        if (touch.isPressed()) {
            resetEncoders();
        }
        motors.setPower(profile.profiledMovement(target, getPosition()),0);
        motors.setPower(profile.profiledMovement(target, getPosition()),1);
    }

    public double getError(){
        return target - getPosition();
    }

    public double getPosition() { return motors.getPosition(0) + offset; }

    public boolean isTimeDone() { return profile.getProfileDuration() < profile.getCurrentTime(); }
    public boolean isPositionDone() { return Math.abs(getError()) < 10; }

    public void setMotionConstraints(double maxVel, double maxAccel, double maxJerk){
        profile.setMotionConstraints(maxVel, maxAccel, maxJerk);
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf, double limit){
        profile.setPIDcoeffs(Kp, Kd, Ki, Kf, limit);
    }

    public double getMotionTarget(){
        return -profile.getMotionTarget();
    }

    public double getTarget() { return -target; }

    public double getMotionTime() { return profile.getCurrentTime(); }

    public void resetEncoders(){
        motors.motors[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.motors[1].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        offset = 0;
    }

    public void addOffset(double offset) {
        this.offset = offset;
    }

    public void zero(){
        moveTo(zero);
        currentHeight = zero;
    }

    public void transfer(){
        moveTo(transfer);
        currentHeight = transfer;
    }

    public void mid(){
        moveTo(mid);
        currentHeight = mid;
    }

    public void high(){
        moveTo(high);
        currentHeight = high;
    }

    public double returnPole() {
        return currentHeight;
    }

}
