package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utility.MotorGroup;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

public class HorizontalSlide {

    //TODO make one motor
    private final MotorGroup motors;
    private double target;
    private final TouchSensor touch;
    private final RunMotionProfile profile = new RunMotionProfile(60000,70000,80000,0.1,0,1,0.2, 1);

    public static final double out = 1700, mid = 300, transfer = 100, in = 0;
    private double currentHeight = in, offset = 0;

    // 0-1100

    public HorizontalSlide(HardwareMap hardwareMap){
        DcMotorEx leftSlide = hardwareMap.get(DcMotorEx.class,"Lslide");
        DcMotorEx rightSlide = hardwareMap.get(DcMotorEx.class,"Rslide");
        touch = hardwareMap.get(TouchSensor.class, "Htouch");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = new MotorGroup(leftSlide,rightSlide);
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
        moveTo(in);
        currentHeight = in;
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
        moveTo(out);
        currentHeight = out;
    }

    public double returnPole() {
        return currentHeight;
    }

}
