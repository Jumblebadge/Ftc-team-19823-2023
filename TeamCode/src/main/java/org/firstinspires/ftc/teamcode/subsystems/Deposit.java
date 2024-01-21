package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.*;
import org.firstinspires.ftc.teamcode.maths.mathsOperations;
import org.opencv.core.Point;

public class Deposit {

    //175, 0.61
    //120, 0.5
    //0, 0.22
    //0.34-0.1

    public static final Point score = new Point(45,0.22), mid = new Point(165,0.5), transfer = new Point(-45, 0.61);
    private final DualServo swing;
    private final ServoImplEx end, latch;
    //private final EncoderServo end;
    private final VerticalSlide slide;
    private final Telemetry telemetry;
    private Point target, current = mid;

    //deposit is end 0.4, swing 0.85

    //end transfer is 1, 0 is deposit
    //latch 1 is closed, 0 is open
    //swing 0-1
    //joystick bounds are 0.5-0.9
    public Deposit(HardwareMap hardwareMap, Telemetry telemetry) {
        ServoImplEx swingL = hardwareMap.get(ServoImplEx.class, "swingL");
        ServoImplEx swingR = hardwareMap.get(ServoImplEx.class, "swingR");

        //swingL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        //swingR.setPwmRange(new PwmControl.PwmRange(500, 2500));

        end = hardwareMap.get(ServoImplEx.class, "endS");
        end.setPwmRange(new PwmControl.PwmRange(500, 2500));
        //AnalogInput endEncoder = hardwareMap.get(AnalogInput.class, "endE");

        latch = hardwareMap.get(ServoImplEx.class, "Dlatch");
        latch.setPwmRange(new PwmControl.PwmRange(500,2500));

        swing = new DualServo(swingL, swingR);
        //end = new EncoderServo(endServo, endEncoder);
        slide = new VerticalSlide(hardwareMap);

        this.telemetry = telemetry;
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

    public void toggleLatch(boolean active) {
        if (active) latch.setPosition(1);
        else latch.setPosition(0);
    }

    public void score(double adjust) {
        double y = score.y + adjust * 0.12;
        swing.setPosition(y);
        end.setPosition(375 * y - 37.5);
        target = score;
        telemetry.addData("ineair",375 * (score.y + adjust * 0.12) - 37.5);
    }

    public void transfer() {
        swing.setPosition(transfer.y);
        end.setPosition(transfer.x);
        target = transfer;
    }

    public void mid() {
        swing.setPosition(mid.y);
        end.setPosition(mid.x);
        target = mid;
    }

    public void update() {
        //isAt();
        //end.update((target == score && current == transfer) || (target == transfer && current == score));
        telemetry.addData("??", target == score && current == transfer || target == transfer && current == score);
        slide.update();
    }
/*
    public void isAt() {
        if (mathsOperations.equals(endPosition(),mid.x,5)) {
            current = mid;
        }
        if (mathsOperations.equals(endPosition(),transfer.x,5)) {
            current = transfer;
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
