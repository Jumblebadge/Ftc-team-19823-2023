package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;


@Config
@TeleOp(name="mecnuemaen", group="Linear Opmode")
public class godMecanum extends LinearOpMode {

    // p6, i15, l0.1

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the mecanum
        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap);
        Deposit deposit = new Deposit(hardwareMap);

        ButtonDetector game2b = new ButtonDetector();

        ElapsedTime hztimer = new ElapsedTime();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            drive.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, Math.pow(gamepad1.right_stick_x,3));

            if (gamepad1.x) {
                game2b.toFalse();
                deposit.transfer();
            }
            else if (gamepad1.a) {
                game2b.toFalse();
                deposit.mid();
            }
            else if (game2b.toggle(gamepad1.b)) {
                deposit.score(gamepad1.right_stick_y);
            }

            telemetry.addData("hz",1/hztimer.seconds());
            hztimer.reset();
            telemetry.update();
        }
    }
}
