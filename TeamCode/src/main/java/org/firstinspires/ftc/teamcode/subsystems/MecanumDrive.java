package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utility.DcMotorExW;
import org.firstinspires.ftc.teamcode.utility.StandardTrackingWheelLocalizer;

public class MecanumDrive implements Driveable{

    final private IMU imu;
    final private StandardTrackingWheelLocalizer odo;
    final private DcMotorExW frontRight, frontLeft, backLeft, backRight;
    final private Telemetry telemetry;
    final private boolean isAuto;

    public MecanumDrive(Telemetry telemetry, HardwareMap hardwareMap, boolean isAuto){
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

        odo = new StandardTrackingWheelLocalizer(hardwareMap);

        this.telemetry = telemetry;
        this.isAuto = isAuto;
    }

    public void drive(double x, double y, double rot){

        //Update heading of robot
        Pose2d pose = odo.getPoseEstimate();
        double heading;

        if (isAuto)  heading = pose.getHeading();
        else heading = imu.getHeadingInRadians();

        double x1 = Math.cos(heading) * x - Math.sin(heading) * y;
        double y1 = Math.sin(heading) * x + Math.cos(heading) * y;

        double highest = Math.max(Math.abs(x1) + Math.abs(y1) + Math.abs(rot), 1);

        frontRight.setPower((y1 - x1 - rot) / highest);
        frontLeft.setPower((y1 + x1 + rot) / highest);
        backRight.setPower((y1 + x1 - rot)/ highest);
        backLeft.setPower((y1 - x1 + rot)/ highest);

        telemetry.addData("imu", imu.getHeadingInDegrees());
        telemetry.addData("enc1",frontRight.getCurrentPosition());
        telemetry.addData("enc2",backRight.getCurrentPosition());
        telemetry.addData("enc3",frontLeft.getCurrentPosition());

    }

    public void resetIMU() {
        imu.resetIMU();
    }

    public double getHeading() {
        return imu.getHeadingInDegrees();
    }

    public Pose2d getPose() {
        odo.update();
        return odo.getPoseEstimate();
    }
}
