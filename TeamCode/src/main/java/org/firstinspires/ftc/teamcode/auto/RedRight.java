package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.maths.GVF;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@Autonomous(name="Red Left", group="Linear Opmode")
public class RedRight extends LinearOpMode {

    //Initialize FTCDashboard
    FtcDashboard dashboard;

    double taskNumber = 0, targetHeading = 180;
    private double nanoTime = 0, hz = 0, count = 0;

    enum apexStates {
        SPIKE,
        CYCLE,
        IDLE
    }

    apexStates apexstate = apexStates.SPIKE;
    Pose2d pose = new Pose2d(-45,-60,90 / (180 / Math.PI));

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

        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, true);

        drive.setPoseEstimate(new Pose2d(3.5,-60,90 / (180 / Math.PI)));
        Deposit deposit = new Deposit(hardwareMap);
        Intake intake = new Intake(hardwareMap);


        //Initialize FTCDashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        //Create objects for the classes we use
        GVF gvf = new GVF(dashboard, PathList.RedRightPathToSpike, 2, 9, 1, telemetry);

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

            Vector2d gvfOut = gvf.output(new Vector2d(pose.getX(), pose.getY()));
            drive.drive(gvfOut.getX(), gvfOut.getY(), gvf.headingOut(drive.getHeadingInDegrees(),targetHeading));

            switch(apexstate){
                case SPIKE:
                    if (detected == HSVDetectElement.State.LEFT) targetHeading = 90;
                    else if (detected == HSVDetectElement.State.RIGHT) targetHeading = -90;
                    else targetHeading = 180;
                    if (gvf.isDone() && taskNumber == 0) {
                        intake.eject();
                        taskNumber++;
                        goofytimer.reset();
                    }
                    if (taskNumber == 1 && goofytimer.seconds() > 3) {
                        intake.off();
                        gvf.setPath(PathList.RedRightSpikeToStack, 2.25, 9, 0.5);
                        taskNumber = 0;
                        targetHeading = 90;
                        apexstate = apexStates.CYCLE;
                    }
                    break;

                case CYCLE:
                    if (gvf.isDone() && taskNumber == 0) {
                        intake.on();
                        taskNumber++;
                        goofytimer.reset();
                    }
                    break;
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
            telemetry.addData("guess",PathList.RedLeftPathToSpike.getPoint(PathList.RedLeftPathToSpike.guessT));
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
