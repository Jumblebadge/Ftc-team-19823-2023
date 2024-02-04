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
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.HorizontalSlide;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlide;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;

import java.lang.reflect.Method;
import java.util.List;


@Config
@TeleOp(name="mecnuemaen", group="Linear Opmode")
public class godMecanum extends LinearOpMode {

    FtcDashboard dashboard;

    private double rotation, heading, nanoTime = 0, intakePower = 0, hz = 0, count = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //to swerve the mecanum
        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, false);
        Deposit deposit = new Deposit(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ButtonDetector headingPIDtoggle  = new ButtonDetector();
        ButtonDetector setHeadingTarget = new ButtonDetector();
        ButtonDetector headingTo270 = new ButtonDetector();
        ButtonDetector headingTo180 = new ButtonDetector();
        ButtonDetector headingTo90 = new ButtonDetector();
        ButtonDetector headingTo0 = new ButtonDetector();
        ButtonDetector intakeReverse = new ButtonDetector();
        ButtonDetector depositMovement = new ButtonDetector();
        ButtonDetector depositLatch = new ButtonDetector();
        ButtonDetector canopeeToggle = new ButtonDetector();

        ButtonDetector rumble = new ButtonDetector();

        ElapsedTime swingTimer = new ElapsedTime();
        ElapsedTimeW test = new ElapsedTimeW();

        PIDcontroller headingPID = new PIDcontroller(0.14,0.001,0,1.25,0.1);

        //Bulk sensor reads
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }
        
        deposit.resetEncoders();
        intake.resetEncoders();

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) { hub.clearBulkCache(); }

            //driving gamepad
            if (headingPIDtoggle.toggle(gamepad1.a)) {
                rotation = headingPID.pidOut(AngleUnit.normalizeDegrees(heading - drive.getHeadingInDegrees()));
            }
            else { rotation = 0; }

            if (setHeadingTarget.risingEdge(gamepad1.x)) {
                heading = drive.getHeading();
            }

            if (headingTo0.risingEdge(gamepad1.dpad_up)) {
                heading = 0;
                headingPIDtoggle.toTrue();
            }
            if (headingTo90.risingEdge(gamepad1.dpad_right)) {
                heading = 90;
                headingPIDtoggle.toTrue();
            }
            if (headingTo180.risingEdge(gamepad1.dpad_down)) {
                heading = 180;
                headingPIDtoggle.toTrue();
            }
            if (headingTo270.risingEdge(gamepad1.dpad_left)) {
                heading = -90;
                headingPIDtoggle.toTrue();
            }


            if (gamepad1.b) {
                drive.resetIMU();
            }

            drive.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, rotation + Math.pow(gamepad1.right_stick_x,3));

            if (gamepad2.a) {
                deposit.in();
                intake.in();
            }
            if (gamepad2.b) {
                deposit.mid1();
            }
            if (gamepad2.x) {
                deposit.mid2();
                intake.mid1();
            }
            if (gamepad2.y) {
                //deposit.out();
            }

            if (gamepad2.dpad_down) {
                intake.in();
            }
            if (gamepad2.dpad_right) {
                intake.mid1();
            }
            if (gamepad2.dpad_left) {
                intake.mid2();
            }
            if (gamepad2.dpad_up) {
                intake.out();
            }
            intakePower = gamepad2.left_trigger/1.5;
            if (intakeReverse.toggle(gamepad2.left_bumper)) {
                intakePower *= -1;
            }
            intake.setIntakePower(intakePower);

            if (depositMovement.toggle(gamepad2.right_bumper)) {
                deposit.setSwingPosition(deposit.SWING_OUT);
                deposit.setEndPosition(deposit.END_OUT);
                swingTimer.reset();
            }
            else {
                deposit.setEndPosition(deposit.END_IN);
                if (swingTimer.seconds() > 0.2) {
                    if (/*intake.currentState() == HorizontalSlide.in && */deposit.currentState() == VerticalSlide.in /*&& intake.isSlideDone()*/ && deposit.isSlideDone()) {
                        deposit.setSwingPosition(deposit.SWING_TRANSFER);
                    }
                    else deposit.setSwingPosition(deposit.SWING_WAIT);
                }
            }
            telemetry.addData("intake",intake.isSlideDone());
            telemetry.addData("deposit", deposit.isSlideDone());

            //intake.toggleLatch(intakeLatch.toggle(gamepad2.back));
            intake.toggleCanopee(canopeeToggle.toggle(gamepad2.left_stick_button));
            deposit.toggleLatch(depositLatch.toggle(gamepad2.triangle));

            if (rumble.risingEdge(gamepad2.triangle)) {
                if (deposit.latchPosition() > 0.5)  gamepad2.rumble(100);
            }


            intake.update();
            deposit.update();

            double nano = System.nanoTime();
            hz += (1000000000 / (nano - nanoTime));
            count++;
            telemetry.addData("hz", hz / count);
            nanoTime = nano;

            telemetry.addData("slide", intake.getPosition());
            telemetry.addData("test",test.seconds());
            telemetry.addData("pise",drive.getPose().toString());
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
