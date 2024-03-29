package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;
import org.firstinspires.ftc.teamcode.utility.ServoImplExW;

public class Intake {

    //TODO make this extensive

    private final DcMotorExW intake;
    private final double intakePower = 1;
    HorizontalSlide slide;
    ServoImplExW latch, canopee;
    public final double LATCH_OPEN = 0.35, LATCH_CLOSED = 0.45;
    public final double CANOPEE_DOWN = 0.4, CANOPEE_UP = 0.575;
    private double last = HorizontalSlide.in;
    private ElapsedTimeW rollerTimer = new ElapsedTimeW();

    //canopee 0 down, 0.4 up, 0.? 5stack
    //latch 0.3 open, 0.45 closed

    public Intake(HardwareMap hardwareMap){
        intake = new DcMotorExW(hardwareMap.get(DcMotorEx.class, "intake"));
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        latch = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "Ilatch"));
        latch.setPwmRange(new PwmControl.PwmRange(500, 2500));

        canopee = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "canopee"));

        slide = new HorizontalSlide(hardwareMap);
        slide.resetEncoders();
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
        if (slide.returnState() == HorizontalSlide.in && Deposit.isTransfer) {
            latch.setPosition(LATCH_OPEN);
        }
        else latch.setPosition(LATCH_CLOSED);
    }

    public void setLatchPosition(double target) { latch.setPosition(target); }

    public void setCanopeePosition(double target) { canopee.setPosition(target); }

    public double getCanopeePosition() { return canopee.getPosition(); }

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

    public void setIntakePower(double power) {
        if (last == HorizontalSlide.in && currentState() != HorizontalSlide.in) {
            rollerTimer.reset();
            last = currentState();
        }
        if (rollerTimer.seconds() > 0.25) {
            intake.setPower(power);
            last = currentState();
        }
        else {
            intake.setPower(0);
            last = currentState();
        }
    }

    public void in() { slide.in(); }

    public void mid1() { slide.mid1(); }

    public void mid2() { slide.mid2(); }

    public void out() { slide.out(); }

    public double currentState() { return slide.returnState(); }


}
