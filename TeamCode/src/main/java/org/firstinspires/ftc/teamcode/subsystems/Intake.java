package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Intake {

    //TODO make this extensive

    private final DcMotorEx intake;
    private final double intakePower = 1;
    HorizontalSlide slide;
    ServoImplEx latch, canopee;

    //canopee 0.05 down, 0.6 up, 0.35 5stack
    //latch 0.6 open, 0.825 closed

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        latch = hardwareMap.get(ServoImplEx.class, "Ilatch");
        latch.setPwmRange(new PwmControl.PwmRange(500, 2500));

        canopee = hardwareMap.get(ServoImplEx.class, "canopee");
        canopee.setPwmRange(new PwmControl.PwmRange(500, 2500));

        slide = new HorizontalSlide(hardwareMap);
    }

    public void on(){
        intake.setPower(intakePower);
    }

    public boolean isSlideDone() { return slide.isTimeDone() || slide.isPositionDone(); }

    public void setSlide(double target) { slide.moveTo(target); }

    public double getPosition() { return slide.getPosition(); }

    public void resetEncoders() { slide.resetEncoders(); }

    public void update() { slide.update(); }

    public void setLatchPosition(double target) { latch.setPosition(target); }

    public void setCanopeePosition(double target) { canopee.setPosition(target); }

    public void eject(){
        intake.setPower(-intakePower);
    }

    public void off(){
        intake.setPower(0);
    }

    public void setIntakePower(double power) { intake.setPower(power); }

}
