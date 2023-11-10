package org.firstinspires.ftc.teamcode.utility;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.utility.RunMotionProfile;

public class EncoderServo {

    private final CRServoImplEx servo;
    private final AnalogInput encoder;
    private final PIDcontroller PID = new PIDcontroller(0.05,0.00002,2,0,0.1);
    private double target;

    public EncoderServo(CRServoImplEx servo, AnalogInput encoder){
        this.servo = servo;
        this.encoder = encoder;
    }

    public void setPIDcoeffs(double Kp, double Kd, double Ki, double Kf, double limit) {
        PID.setPIDgains(Kp, Kd, Ki, Kf, limit);
    }

    public double getPosition() {
        return -AngleUnit.normalizeDegrees(encoder.getVoltage() / 3.3 * 360);
    }

    public double getError(){
        return AngleUnit.normalizeDegrees(target - getPosition());
    }

    public void PWMrelease() {
        servo.setPwmDisable();
    }

    public void setPosition(double position) {
        target = position;
    }

    public void update() {
        servo.setPower(PID.pidOut(AngleUnit.normalizeDegrees(target - getPosition())));
    }

}
