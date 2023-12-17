package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlide;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlide;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;


@Config
@TeleOp(name="mecnuemaen", group="Linear Opmode")
public class godMecanum extends LinearOpMode {

    FtcDashboard dashboard;

    private double rotation, heading;
    public static double target=0, Kp=0, Kd=0, Ki=0, Kf=0, Kl=1000;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //to swerve the mecanum
        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, false);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ButtonDetector game2dl  = new ButtonDetector();
        ButtonDetector game2rb  = new ButtonDetector();
        ButtonDetector game1a   = new ButtonDetector();
        ButtonDetector game1rb  = new ButtonDetector();
        ButtonDetector game1lb  = new ButtonDetector();

        ElapsedTime hztimer = new ElapsedTime();

        PIDcontroller headingPID = new PIDcontroller(0.14,0.001,0,1.25,0.1);

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            //driving gamepad

            if (game1a.toggle(gamepad1.a)) {
                rotation = headingPID.pidOut(AngleUnit.normalizeDegrees(heading - drive.getHeadingInDegrees()));
            }
            else { rotation = 0; }

            if (game1rb.risingEdge(gamepad1.right_bumper)) {
                heading = drive.getHeading();
            }

            if (game1lb.risingEdge(gamepad1.left_bumper)) {
                heading = 0;
                game1a.toTrue();
            }

            if (gamepad1.b) {
                drive.resetIMU();
            }

            drive.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, rotation + Math.pow(gamepad1.right_stick_x,3));

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose2d pose = drive.getPose();
            drawRobot(fieldOverlay, drive.getPose());
            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));

            dashboard.sendTelemetryPacket(packet);


            //subsystem gamepad
            /**
            if (gamepad2.dpad_right) {
                game2dl.toFalse();
                deposit.transfer();
            }
            else if (gamepad2.dpad_down) {
                game2dl.toFalse();
                deposit.mid();
            }
            else if (game2dl.toggle(gamepad2.b)) {
                deposit.score(gamepad1.right_stick_y);
            }

            if (game2rb.toggle(gamepad2.right_bumper)) {
                intake.on();
            }
            else if (gamepad1.left_bumper) {
                intake.eject();
            }
            else {
                intake.off();
            }
            **/
            telemetry.addData("hz",1/hztimer.seconds());
            telemetry.addData("dulgk",target);
            telemetry.addData("pise",drive.getPose().toString());
            hztimer.reset();
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
