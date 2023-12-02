package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;

public class MecanumDrive {

    final private IMU imu;
    final private DcMotorExW frontRight, frontLeft, backLeft, backRight;
    final private Telemetry telemetry;

    public MecanumDrive(Telemetry telemetry, HardwareMap hardwareMap){
        frontRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"frontRight"));
        frontLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"frontLeft"));
        backLeft = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"backLeft"));
        backRight = new DcMotorExW(hardwareMap.get(DcMotorEx.class,"backRight"));

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
        double heading = imu.getHeadingInDegrees();

        double x1 = Math.cos(Math.toRadians(heading)) * x - Math.sin(Math.toRadians(heading)) * -y;
        double y1 = Math.sin(Math.toRadians(heading)) * x + Math.cos(Math.toRadians(heading)) * -y;

        double highest = Math.max(Math.abs(x1) + Math.abs(y1) + Math.abs(rot), 1);

        frontRight.setPower((y1 - x1 - rot) / highest);
        frontLeft.setPower((y1 + x1 + rot) / highest);
        backRight.setPower((y1 + x1 - rot)/ highest);
        backLeft.setPower((y1 - x1 + rot)/ highest);

        telemetry.addData("imu", imu.getHeadingInDegrees());

    }

    public void resetIMU() {
        imu.resetIMU();
    }


    public double getHeading() {
        return imu.getHeadingInDegrees();
    }
}
