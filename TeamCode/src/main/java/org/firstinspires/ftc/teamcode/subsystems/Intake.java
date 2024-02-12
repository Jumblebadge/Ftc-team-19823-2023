package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.ServoImplExW;

public class Intake {

    //TODO make this extensive

    private final DcMotorExW intake;
    private final double intakePower = 1;
    HorizontalSlide slide;
    ServoImplExW latch, canopee;
    public final double LATCH_OPEN = 0.3, LATCH_CLOSED = 0.45;
    public final double CANOPEE_DOWN = 0.425, CANOPEE_UP = 0.775;

    //canopee 0 down, 0.4 up, 0.? 5stack
    //latch 0.3 open, 0.45 closed

    public Intake(HardwareMap hardwareMap){
        intake = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "intake"));
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        latch = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "Ilatch"));
        latch.setPwmRange(new PwmControl.PwmRange(500, 2500));

        canopee = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "canopee"));
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

    public void update() {
        slide.update();
        if (slide.returnState() != HorizontalSlide.in) latch.setPosition(LATCH_CLOSED);
        else if (isSlideDone() && slide.returnState() == HorizontalSlide.in) latch.setPosition(LATCH_OPEN);
    }

    public void setLatchPosition(double target) { latch.setPosition(target); }

    public void setCanopeePosition(double target) { canopee.setPosition(target); }

    public void toggleCanopee(boolean active) {
        if (active) canopee.setPosition(CANOPEE_DOWN);
        else canopee.setPosition(CANOPEE_UP);
    }

    public void eject(){
        intake.setPower(-intakePower);
    }

    public void off(){
        intake.setPower(0);
    }

    public void toggleLatch(boolean active) {
        if (active) latch.setPosition(LATCH_OPEN);
        else latch.setPosition(LATCH_CLOSED);
    }

    public void setIntakePower(double power) { intake.setPower(power); }

    public void in() { slide.in(); }

    public void mid1() { slide.mid1(); }

    public void mid2() { slide.mid2(); }

    public void out() { slide.out(); }

    public double currentState() { return slide.returnState(); }


}
