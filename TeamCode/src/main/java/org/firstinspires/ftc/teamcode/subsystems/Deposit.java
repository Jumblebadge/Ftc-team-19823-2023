package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.utility.EncoderServo;
import org.firstinspires.ftc.teamcode.utility.DualServo;
import org.opencv.core.Point;

public class Deposit {

    //175, 0.61
    //120, 0.5
    //0, 0.22
    //0.34-0.1

    public static final Point score = new Point(45,0.22), mid = new Point(165,0.5), transfer = new Point(-45, 0.61);
    private final DualServo swing;
    private final EncoderServo end;

    public Deposit(HardwareMap hardwareMap) {
        ServoImplEx swingL = hardwareMap.get(ServoImplEx.class, "swingL");
        ServoImplEx swingR = hardwareMap.get(ServoImplEx.class, "swingR");

        swingL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        swingR.setPwmRange(new PwmControl.PwmRange(500, 2500));

        CRServoImplEx endServo = hardwareMap.get(CRServoImplEx.class, "endS");
        AnalogInput endEncoder = hardwareMap.get(AnalogInput.class, "endE");

        swing = new DualServo(swingL, swingR);
        end = new EncoderServo(endServo, endEncoder);
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
        end.setPosition(375 * y - 82.5);
    }

    public void transfer() {
        swing.setPosition(transfer.y);
        end.setPosition(transfer.x);

    }

    public void mid() {
        swing.setPosition(mid.y);
        end.setPosition(mid.x);
    }

    public void update() {
        end.update();
    }

}
