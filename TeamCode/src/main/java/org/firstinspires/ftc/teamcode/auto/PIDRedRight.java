package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.maths.GoToPoint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.vision.HSVDetectElement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;


@Config
@Autonomous(name="Red Right", group="Linear Opmode")
public class PIDRedRight extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    double lastX = 0.0001, lastY = 0.0001;
    public static double heading;

    Pose2d pose = new Pose2d(0.01,0.01,0);
    Pose2d temp = new Pose2d(0,0,0);
    Pose2d targetPose = new Pose2d(0,0,0);
    double taskNumber = 0;

    enum apexStates {
        SPIKE,
        DEPOSIT,
        PARK
    }

    apexStates apexstate = apexStates.SPIKE;

    GoToPoint auto;

    VisionProcessor processor;
    VisionPortal portal;

    ElapsedTime goofytimer = new ElapsedTime();
    ElapsedTime hztimer = new ElapsedTime();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Waiting for start");

        processor = new HSVDetectElement();
        portal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam"), processor);

        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, true);
        Deposit deposit = new Deposit(hardwareMap, telemetry);

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();

        //Create objects for the classes we use
        auto = new GoToPoint(drive,telemetry,dashboard);

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("seen", HSVDetectElement.returnDetected());
            telemetry.update();
            sleep(20);
        }

        waitForStart();
        HSVDetectElement.State detected = HSVDetectElement.returnDetected();
        apexstate = apexStates.SPIKE;
        targetPose = new Pose2d(0.1,0.1,0.001);
        goofytimer.reset();
        drive.resetIMU();


        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            pose = drive.getPose();
            runPoint(targetPose);
            switch (apexstate){

                case SPIKE:
                    double horizontalShift = (detected == HSVDetectElement.State.MID ? 0 : (detected == HSVDetectElement.State.LEFT ? -14.5 : 14.5));
                    if (auto.isDone() && taskNumber == 0) {
                        targetPose = new Pose2d(28,0,0);
                        taskNumber++;
                    }
                    if (auto.isPositionDone() && taskNumber == 1) {
                        targetPose = new Pose2d((detected == HSVDetectElement.State.MID ? 38 : 28), horizontalShift, 0);
                        taskNumber++;
                    }
                    if (auto.isPositionDone() && taskNumber == 2) {
                        targetPose = new Pose2d(28, 0, 0);
                        taskNumber++;
                    }
                    if (auto.isPositionDone() && taskNumber == 3) {
                        targetPose = new Pose2d(12, 0, 1.57);
                        taskNumber++;
                    }
                    if (auto.isPositionDone() && taskNumber == 4) {
                        targetPose = new Pose2d(12, 35.5, 1.57);
                        taskNumber++;
                    }
                    if (auto.isPositionDone() && taskNumber == 5) {
                        goofytimer.reset();
                        //apexstate = apexStates.DEPOSIT;
                        //taskNumber = 0;
                    }
                    break;

                case DEPOSIT:
                    if (auto.isPositionDone() && taskNumber == 0) {
                        targetPose = new Pose2d((detected == HSVDetectElement.State.MID ? 12 : (detected == HSVDetectElement.State.LEFT ? 16 : 8)), 42, 1.57);
                        taskNumber++;
                    }
                    if (auto.isPositionDone() && taskNumber == 1) {
                        deposit.setSlide(400);
                        deposit.score(0);
                        if (deposit.isSlideDone()) {
                            deposit.disableLatch();
                            taskNumber++;
                        }
                    }
                    if (auto.isPositionDone() && taskNumber == 2 && deposit.isSlideDone()) {

                    }
                    break;

                case PARK:
                    //drive to park position
                    if (auto.isPositionDone() && taskNumber == 0) {
                        targetPose = new Pose2d(24, 42, 0);
                        taskNumber++;
                    }
                    if (auto.isPositionDone() && taskNumber == 1) {
                        targetPose = new Pose2d(24, 48, 0);
                    }
                    break;

            }

            heading = Math.toDegrees(pose.getHeading());
            telemetry.addData("apexstate",apexstate.toString());
            telemetry.addData("hz",1/hztimer.seconds());
            hztimer.reset();
            telemetry.update();


        }


        //if(detectedTag != null)
        {
            telemetry.addLine("tag:\n");
            //webcamStuff.tagToTelemetry(detectedTag);
        }
        //else
        {
            telemetry.addLine("never seen tag");
        }

    }

    public void runPoint(Pose2d desiredPose){
        if (lastX != desiredPose.getX() || lastY != desiredPose.getY()) {
            lastX = desiredPose.getX();
            lastY = desiredPose.getY();
            temp  = pose;
            auto.driveToPoint(pose,desiredPose,temp,true);
        }
        else{ lastX = desiredPose.getX(); lastY = desiredPose.getY(); }
        auto.driveToPoint(pose,desiredPose,temp,false);
    }


}