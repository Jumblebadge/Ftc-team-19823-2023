package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.myDcMotorEx;

public class MecanumDrive {

    final private IMU imu;
    final private myDcMotorEx frontRight, frontLeft, backLeft, backRight;
    final private Telemetry telemetry;

    double heading;
    double imuOffset = 0;

    public MecanumDrive(Telemetry telemetry, HardwareMap hardwareMap){
        frontRight = new myDcMotorEx(hardwareMap.get(DcMotorEx.class,"mod1m1"));
        frontLeft = new myDcMotorEx(hardwareMap.get(DcMotorEx.class,"mod1m2"));
        backLeft = new myDcMotorEx(hardwareMap.get(DcMotorEx.class,"mod2m1"));
        backRight = new myDcMotorEx(hardwareMap.get(DcMotorEx.class,"mod2m2"));

        frontRight.setPowerThresholds(0.05,0);
        frontLeft.setPowerThresholds(0.05,0);
        backLeft.setPowerThresholds(0.05,0);
        backRight.setPowerThresholds(0.05,0);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new IMU(hardwareMap);

        this.telemetry = telemetry;
    }

    public void drive(double x, double y, double rot){

        //Update heading of robot
        heading = imu.getHeadingInDegrees();

        double x1 = 1*(Math.cos(Math.toRadians(heading)) * x - Math.sin(Math.toRadians(heading)) * y);
        double y1 = -(Math.sin(Math.toRadians(heading)) * x + Math.cos(Math.toRadians(heading)) * y);

        double highest = Math.max(Math.abs(x1) + Math.abs(y1) + Math.abs(rot), 1);

        frontRight.setPower((y1 - x1 - rot) / highest);
        frontLeft.setPower((y1 + x1 + rot) / highest);
        backRight.setPower((y1 + x1 - rot)/ highest);
        backLeft.setPower((y1 - x1 + rot)/ highest);


    }

    public void rotateKids(double angle) {
        this.imuOffset = angle;
    }

    public void resetIMU() {
        imu.resetIMU();
    }


    public double getHeading() {
        return imu.getHeadingInDegrees();
    }
}
