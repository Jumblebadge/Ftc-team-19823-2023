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
    private final EncoderServo end;
    Telemetry telemetry;
    private Point target, current = mid;

    public Deposit(HardwareMap hardwareMap, Telemetry telemetry) {
        ServoImplEx swingL = hardwareMap.get(ServoImplEx.class, "swingL");
        ServoImplEx swingR = hardwareMap.get(ServoImplEx.class, "swingR");

        swingL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        swingR.setPwmRange(new PwmControl.PwmRange(500, 2500));

        CRServoImplEx endServo = hardwareMap.get(CRServoImplEx.class, "endS");
        AnalogInput endEncoder = hardwareMap.get(AnalogInput.class, "endE");

        swing = new DualServo(swingL, swingR);
        end = new EncoderServo(endServo, endEncoder);

        this.telemetry = telemetry;
    }

    public void setSwingPosition(double target) {
        swing.setPosition(target);
    }

    public void setEndPosition(double target) {
        end.setPosition(target);
    }

    public double swingPosition(){
        return  swing.getPosition();
    }

    public double endPosition(){
        return  end.getPosition();
    }

    public void PWMrelease() {
        swing.PWMrelease();
        end.PWMrelease();
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
        isAt();
        end.update((target == score && current == transfer) || (target == transfer && current == score));
        telemetry.addData("??", target == score && current == transfer || target == transfer && current == score);
    }

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

    public void setR(double r){
        end.setR(r);
    }

}
