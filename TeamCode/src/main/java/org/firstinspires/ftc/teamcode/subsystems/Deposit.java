package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.*;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.opencv.core.Point;

public class Deposit {

    private final DualServo swing;
    private final ServoImplExW end, latch;
    private final ElapsedTimeW latchTime = new ElapsedTimeW();
    private final VerticalSlide slide;
    public final double LATCH_CLOSED = 1, LATCH_OPEN = 0;
    public final double END_IN = 1, END_OUT = 0.2;
    public final double SWING_OUT = 0.7, SWING_TRANSFER = 0.165, SWING_WAIT = 0.25;
    public final double SECONDARY_SWING_OUT = 0.775, SECONDARY_END_OUT = 0.255;
    public static boolean isTransfer = true;

    public Deposit(HardwareMap hardwareMap) {
        ServoImplExW swingL = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "swingL"));
        ServoImplExW swingR = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "swingR"));

        //swingL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        //swingR.setPwmRange(new PwmControl.PwmRange(500, 2500));

        end = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "endS"));
        end.setPwmRange(new PwmControl.PwmRange(500, 2500));
        //AnalogInput endEncoder = hardwareMap.get(AnalogInput.class, "endE");

        latch = new ServoImplExW(hardwareMap.get(ServoImplEx.class, "Dlatch"));
        latch.setPwmRange(new PwmControl.PwmRange(500,2500));

        swing = new DualServo(swingL, swingR);
        //end = new EncoderServo(endServo, endEncoder);
        slide = new VerticalSlide(hardwareMap);
    }

    public boolean isSlideDone() { return slide.isTimeDone() || slide.isPositionDone(); }

    public double getPosition() { return slide.getPosition(); }

    public void setSlide(double target) { slide.moveTo(target); }

    public void setSwingPosition(double target) { swing.setPosition(target); }

    public void setLatchPosition(double target) { latch.setPosition(target); }

    public void setEndPosition(double target) { end.setPosition(target); }

    public double swingPosition(){
        return  swing.getPosition();
    }

    public void resetEncoders() { slide.resetEncoders(); }

    public void disabledPIDsetPower(double power) {
        slide.disabledPIDsetPower(power);
    }

    public double endPosition(){ return  end.getPosition(); }

    public void PWMrelease() {
        swing.PWMrelease();
        end.setPwmDisable();
    }

    public void setLatch(double position) {
        latch.setPosition(position);
    }

    public void activateLatch() {
        setLatch(0.5);
    }

    public void disableLatch() {
        setLatch(0.4);
    }

    public void toggleLatch(boolean open) {
        if (isTransfer) latch.setPosition(LATCH_OPEN);
        else if (open) {
            latch.setPosition(LATCH_OPEN);
            latchTime.reset();
        }
        else if (latchTime.seconds() > 1){
            latch.setPosition(LATCH_CLOSED);
        }
    }

    public void toggleLatch(boolean open, double time) {
        if (isTransfer) latch.setPosition(LATCH_OPEN);
        else if (open) {
            latch.setPosition(LATCH_OPEN);
            latchTime.reset();
        }
        else if (latchTime.seconds() > time){
            latch.setPosition(LATCH_CLOSED);
        }
    }

    public double latchPosition() {
        return latch.getPosition();
    }

    public void in() { slide.in(); }

    public void mid1() { slide.mid1(); }

    public void mid2() { slide.mid2(); }

    public void out() { slide.out(); }

    public void update(boolean endIn) {
        slide.update();
        isTransfer = slide.returnState() == VerticalSlide.in && endIn;
    }

    public double currentState() { return slide.returnState(); }
/*
    public void isAt() {
        if (mathsOperations.equals(endPosition(),mid.x,5)) {
            current = mid;
        }
        if (mathsOperations.equals(endPosition(),mid1.x,5)) {
            current = mid1;
        }
        if (mathsOperations.equals(endPosition(),score.x,5)) {
            current = score;
        }
        telemetry.addData("cerent",current);
    }
*/
    public void setR(double r){
        //end.setR(r);
    }

}
