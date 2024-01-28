package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.maths.CubicPath;
import org.firstinspires.ftc.teamcode.maths.GVF;
import org.firstinspires.ftc.teamcode.maths.GoToPoint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.HSVDetectElement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.List;


@Config
@Disabled
@Autonomous(name="Red Left", group="Linear Opmode")
public class RedLeft extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    double taskNumber = 0;

    enum apexStates {
        SPIKE,
        DEPOSIT,
        PARK
    }

    apexStates apexstate = apexStates.SPIKE;

    GVF gvf;

    VisionProcessor processor;
    VisionPortal portal;

    ElapsedTime goofytimer = new ElapsedTime();
    ElapsedTime hztimer = new ElapsedTime();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Waiting for start");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        processor = new HSVDetectElement();
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam"), processor);

        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, false);

        drive.setPoseEstimate(new Pose2d(-45,-60,90 / (180 / Math.PI)));
        Deposit deposit = new Deposit(hardwareMap);


        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Create objects for the classes we use
        gvf = new GVF(dashboard, PathList.RedLeftPathToSpike, 1.6, 9, 1, telemetry);

        //Bulk sensor reads
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        //Fast loop go brrr

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("seen", HSVDetectElement.returnDetected());
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        HSVDetectElement.State detected = HSVDetectElement.returnDetected();
        apexstate = apexStates.SPIKE;
        goofytimer.reset();
        drive.resetIMU();
        portal.close();


        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            Pose2d pose = drive.getPose();
            Vector2d gvfOut = gvf.output(new Vector2d(pose.getX(), pose.getY()));
            drive.drive(gvfOut.getX(), gvfOut.getY(), gvf.headingOut(drive.getHeadingInDegrees(),180));
            telemetry.addData("pose",pose);
            telemetry.update();



        }
    }
}
