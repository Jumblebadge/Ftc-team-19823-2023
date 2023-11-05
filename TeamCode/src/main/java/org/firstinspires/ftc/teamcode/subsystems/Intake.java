package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final DcMotorEx intake;
    private final double intakePower = 1;

    public Intake(HardwareMap hardwareMap){
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void on(){
        intake.setPower(intakePower);
    }

    public void eject(){
        intake.setPower(-intakePower);
    }

    public void off(){
        intake.setPower(0);
    }

}
