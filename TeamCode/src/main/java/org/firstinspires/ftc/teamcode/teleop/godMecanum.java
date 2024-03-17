package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSlide;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;

import java.util.List;


@Config
@TeleOp(name="mecnuemaen", group="Linear Opmode")
public class godMecanum extends LinearOpMode {

    FtcDashboard dashboard;

    public static double slideTarget = 0;

    private double rotation, heading, nanoTime = 0, intakePower = 0, hz = 0, count = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        //to swerve the mecanum
        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, false);
        Deposit deposit = new Deposit(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Plane plane = new Plane(hardwareMap);

        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        ButtonDetector headingPIDtoggle  = new ButtonDetector();
        ButtonDetector intakeReverse = new ButtonDetector();
        ButtonDetector depositMovement = new ButtonDetector();
        ButtonDetector canopeeToggle = new ButtonDetector();
        ButtonDetector slideModeToggle = new ButtonDetector();
        ButtonDetector testToggle = new ButtonDetector();

        ElapsedTimeW swingTimer = new ElapsedTimeW();

        Gamepad current1 = new Gamepad();
        Gamepad previous1 = new Gamepad();

        Gamepad current2 = new Gamepad();
        Gamepad previous2 = new Gamepad();

        PIDcontroller headingPID = new PIDcontroller(0.14,0.001,0,1.25,0.1);

        //Bulk sensor reads
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        waitForStart();
        while (opModeIsActive()) {
            previous1.copy(current1);
            current1.copy(gamepad1);
            previous2.copy(current2);
            current2.copy(gamepad2);

            //Clear the cache for better loop times (bulk sensor reads)
            for (LynxModule hub : allHubs) hub.clearBulkCache();

            //driving gamepad
            if (headingPIDtoggle.toggle(gamepad1.a)) {
                rotation = headingPID.pidOut(AngleUnit.normalizeDegrees(heading - drive.getHeadingInDegrees()));
            }
            else { rotation = 0; }

            if (current1.x && !previous1.x) {
                heading = drive.getHeading();
            }

            if (current1.dpad_up && !previous1.dpad_up) {
                heading = 0;
                headingPIDtoggle.toTrue();
            }
            if (current1.dpad_right && !previous1.dpad_right) {
                heading = 90;
                headingPIDtoggle.toTrue();
            }
            if (current1.dpad_down && !previous1.dpad_down) {
                heading = 180;
                headingPIDtoggle.toTrue();
            }
            if (current1.dpad_left && !previous1.dpad_left) {
                heading = -90;
                headingPIDtoggle.toTrue();
            }

            if (gamepad1.left_bumper && gamepad1.right_bumper) {
                plane.shoot();
            }
            else plane.hold();


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
            if (intakeReverse.toggle(gamepad2.left_bumper)) intakePower *= -1;
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
            telemetry.addData("game",-gamepad1.left_stick_x);

            intake.toggleCanopee(canopeeToggle.toggle(gamepad2.left_stick_button));
            deposit.toggleLatch(testToggle.toggle(current2.triangle), current2.right_trigger > 0.1 && !(previous2.right_trigger > 0.1));

            if (current2.triangle && !previous2.triangle) {
                if (deposit.latchPosition() > 0.5)  gamepad2.rumble(100);
            }
            intake.update();

            if (slideModeToggle.toggle(gamepad2.options && gamepad2.back)) {
                if (deposit.getPosition() < 3000) deposit.disabledPIDsetPower(-gamepad2.right_stick_y);
            }
            else deposit.update(depositMovement.toggle(gamepad2.right_bumper));

            double nano = System.nanoTime();
            hz = (1000000000 / (nano - nanoTime));
            count++;
            telemetry.addData("hz", hz);
            nanoTime = nano;

            telemetry.addData("slide", deposit.getPosition());
            telemetry.addData("test",intake.getCanopeePosition());
            telemetry.addData("isTransfer",Deposit.isTransfer);
            telemetry.addData("pise",drive.getPose().toString());
            telemetry.addData("slide", deposit.getPosition());
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
