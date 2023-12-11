package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.Bezier;
import org.firstinspires.ftc.teamcode.maths.GoToPoint;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;


@Config
@TeleOp(name="mecnuemaen", group="Linear Opmode")
public class godMecanum extends LinearOpMode {

    FtcDashboard dashboard;
    GoToPoint auto;

    private double heading = 0, rotation = 0;

    double lastX = 0.0001, lastY = 0.0001, turretTarget = 0;
    public static double targetX = 0, targetY = 0, targetHeading = 0;

    Pose2d pose = new Pose2d(0,0,0);
    Pose2d temp = new Pose2d(0,0,0);
    public static Pose2d targetPose = new Pose2d(0,0,0);



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //to swerve the mecanum
        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, false);
        auto = new GoToPoint(drive,telemetry,dashboard);
        //Deposit deposit = new Deposit(hardwareMap, telemetry);
        //Intake intake = new Intake(hardwareMap);

        dashboard = FtcDashboard.getInstance();

        ButtonDetector game2dl  = new ButtonDetector();
        ButtonDetector game2rb  = new ButtonDetector();
        ButtonDetector game1a   = new ButtonDetector();
        ButtonDetector game1rb  = new ButtonDetector();
        ButtonDetector game1lb  = new ButtonDetector();

        ElapsedTime hztimer = new ElapsedTime();

        PIDcontroller headingPID = new PIDcontroller(0.14,0.001,0,1.25,0.1);

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //deposit.transfer();

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            //driving gamepad

            if (game1a.toggle(gamepad1.a)) {
                rotation = headingPID.pidOut(AngleUnit.normalizeDegrees(heading - drive.getHeading()));
            }
            else { rotation = 0; }

            if (game1rb.risingEdge(gamepad1.right_bumper)) {
                heading = drive.getHeading();
            }

            if (game1lb.risingEdge(gamepad1.left_bumper)) {
                heading = 0;
                game1a.toTrue();
            }

            //drive.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, rotation + Math.pow(gamepad1.right_stick_x,3));

            pose = drive.getPose();
            targetPose = new Pose2d(targetX,targetY,targetHeading);
            runPoint(targetPose);

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
            telemetry.addData("pise",drive.getPose().toString());
            hztimer.reset();
            telemetry.update();
        }
    }

    public void runPoint(Pose2d desiredPose){
        if (lastX != desiredPose.getX() || lastY != desiredPose.getY()) {
            lastX = desiredPose.getX();
            lastY = desiredPose.getY();
            temp  = new Pose2d(pose.getX(), pose.getY(),pose.getHeading());
            auto.driveToPoint(pose,desiredPose,temp,true);
        }
        else{ lastX = desiredPose.getX(); lastY = desiredPose.getY(); }
        auto.driveToPoint(pose,desiredPose,temp,false);
    }
}
