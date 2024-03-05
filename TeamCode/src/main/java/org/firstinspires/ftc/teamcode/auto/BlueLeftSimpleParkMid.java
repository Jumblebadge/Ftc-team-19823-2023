package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.maths.GVF;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlide;
import org.firstinspires.ftc.teamcode.utility.CameraShenanigans;
import org.firstinspires.ftc.teamcode.vision.HSVDetectElement;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import java.util.List;


@Config
@Autonomous(name="Blue Left Park Mid", group="Linear Opmode")
public class BlueLeftSimpleParkMid extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    double taskNumber = 0, targetHeading = 180;
    private double nanoTime = 0, hz = 0, count = 0;
    private boolean depositScoring = false, followTangent = true;

    double temp = 0;

    enum apexStates {
        SPIKE,
        CYCLE
    }

    apexStates apexstate = apexStates.SPIKE;
    Pose2d pose = new Pose2d(12,60,90 / (180 / Math.PI));

    ElapsedTime goofytimer = new ElapsedTime();
    ElapsedTime swingTimer = new ElapsedTime();



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Waiting for start");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, true);

        drive.setPoseEstimate(new Pose2d(12,62.75,-90 / (180 / Math.PI)));
        Deposit deposit = new Deposit(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        CameraShenanigans camera = new CameraShenanigans(telemetry, hardwareMap, dashboard);
        camera.enableAprilTag(false);
        camera.enableHSVDetection(true);

        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        //Create objects for the classes we use
        GVF gvf = new GVF(dashboard, BluePathList.RightPathToSpike, 4, 15, 0.5, telemetry);

        //Bulk sensor reads
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        //Fast loop go brrr

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("seen", HSVDetectElement.returnDetected());
            telemetry.update();
            sleep(20);
            if (HSVDetectElement.returnDetected() == HSVDetectElement.State.LEFT && taskNumber == 0) {
                gvf.setPath(BluePathList.RightPathToLeftSpike, 4, 15, 0.5);
            }
            else if (HSVDetectElement.returnDetected() == HSVDetectElement.State.RIGHT && taskNumber == 0) {
                gvf.setPath(BluePathList.RightPathToRightSpike, 4, 15, 0.5);
            }
            else gvf.setPath(BluePathList.RightPathToSpike, 4, 15, 0.5);
            intake.setCanopeePosition(intake.CANOPEE_DOWN);
        }

        waitForStart();
        HSVDetectElement.State detected = HSVDetectElement.returnDetected();
        apexstate = apexStates.SPIKE;
        goofytimer.reset();
        drive.resetIMU();
        intake.setIntakePower(0);
        camera.enableHSVDetection(false);
        taskNumber = 0;
        deposit.toggleLatch(true);


        while (opModeIsActive()) {
            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();
            camera.update();

            Vector2d gvfOut = gvf.output(new Vector2d(pose.getX(), pose.getY()));
            camera.telemetryAprilTag();
            drive.drive(-gvfOut.getX(), -gvfOut.getY(), gvf.headingOut(drive.getHeadingInDegrees(),targetHeading, followTangent, true));

            switch(apexstate){
                case SPIKE:
                    if (detected == HSVDetectElement.State.LEFT) targetHeading = 90;
                    else if (detected == HSVDetectElement.State.RIGHT) targetHeading = -90;
                    else targetHeading = 180;
                    if (gvf.isDone(5, 5) && taskNumber == 0 && goofytimer.seconds() > 3) {
                        //intake.setIntakePower(-0.5);
                        intake.setCanopeePosition(intake.CANOPEE_UP);
                        intake.setCanopeePosition(intake.CANOPEE_DOWN);
                        intake.setCanopeePosition(intake.CANOPEE_UP);
                        taskNumber++;
                        goofytimer.reset();
                    }
                    if (taskNumber == 1 && goofytimer.seconds() > 4) {
                        intake.off();
                        gvf.setPath(BluePathList.LeftSpikeToBoard, 3.5, 22.5, 0.5);
                        taskNumber = 0;
                        targetHeading = -90;
                        camera.enableAprilTag(true);
                        apexstate = apexStates.CYCLE;
                    }
                    break;

                case CYCLE:
                    if (taskNumber == 0 && gvf.isDone(10, 10) && goofytimer.seconds() > 0.25) {
                        if (camera.seen()) {
                            drive.setPoseEstimate(new Pose2d(36 + (18 - camera.tag5Values[1]),36 - camera.tag5Values[0]));
                        }
                        taskNumber++;
                        goofytimer.reset();
                        camera.enableAprilTag(false);
                        if (detected == HSVDetectElement.State.RIGHT) gvf.setPath(BluePathList.BoardAdjustmentLeft, 4, 15, 0.5);
                        else if (detected == HSVDetectElement.State.LEFT) gvf.setPath(BluePathList.BoardAdjustmentRight, 4, 15, 0.5);
                        else gvf.setPath(BluePathList.BoardAdjustment, 4, 15, 0.5);
                    }
                    if (taskNumber == 1 && gvf.isDone(5, 10) && goofytimer.seconds() > 0.25) {
                        taskNumber++;
                        goofytimer.reset();
                        depositScoring = true;
                    }
                    if (taskNumber == 2 && goofytimer.seconds() > 1) {
                        deposit.toggleLatch(false);
                    }
                    if (taskNumber == 2 && goofytimer.seconds() > 2) {
                        depositScoring = false;
                        targetHeading = 0;
                        followTangent = false;
                        gvf.setPath(BluePathList.ParkMid, 4, 7, 0.5);
                        goofytimer.reset();
                        taskNumber++;
                    }
                    break;
            }

            if (depositScoring) {
                temp = 0;
                deposit.setSwingPosition(deposit.SECONDARY_SWING_OUT);
                deposit.setEndPosition(deposit.SECONDARY_END_OUT);
                swingTimer.reset();
            }
            else {
                deposit.setEndPosition(deposit.END_IN);
                temp = 1;
                if (swingTimer.seconds() > 0.2) {
                    if (/*intake.currentState() == HorizontalSlide.in && */deposit.currentState() == VerticalSlide.in /*&& intake.isSlideDone()*/ && deposit.isSlideDone()) {
                        deposit.setSwingPosition(deposit.SWING_TRANSFER);
                        temp = 2;
                    }
                    else deposit.setSwingPosition(deposit.SWING_WAIT);
                }
            }

            pose = drive.getPose();
            TelemetryPacket packet = new TelemetryPacket();
            Canvas canvas = packet.fieldOverlay();
            drawRobot(canvas, pose);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("pose",pose);
            double nano = System.nanoTime();
            hz += (1000000000 / (nano - nanoTime));
            count++;
            telemetry.addData("hz", hz / count);
            nanoTime = nano;
            telemetry.addData("temp",temp);
            telemetry.addData("isdone",gvf.isDone(10, 10));
            telemetry.addData("guess", BluePathList.RightPathToSpike.getPoint(BluePathList.RightPathToSpike.guessT));
            telemetry.addData("y",gvfOut.getY());
            telemetry.addData("x",gvfOut.getX());
            telemetry.update();



        }

    }
    public static void drawRobot(Canvas canvas, Pose2d pose) {
        canvas.strokeCircle(pose.getX(), pose.getY(), 9);
        Vector2d v = pose.headingVec().times(9);
        double x1 = pose.getX() + v.getX() / 2, y1 = pose.getY() + v.getY() / 2;
        double x2 = pose.getX() + v.getX(), y2 = pose.getY() + v.getY();
        canvas.strokeLine(x1, y1, x2, y2);
    }
}
