package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;


@Config
@Autonomous(name="BlueLeft", group="Linear Opmode")
public class TimedBlueLeft extends LinearOpMode {

    private double heading = 0, rotation = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //to swerve the mecanum
        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, false);
        //Deposit deposit = new Deposit(hardwareMap, telemetry);
        //Intake intake = new Intake(hardwareMap);

        ElapsedTime hztimer = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();

        PIDcontroller headingPID = new PIDcontroller(0.14,0.001,0,1.25,0.1);

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //deposit.mid1();

        waitForStart();
        timer.reset();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            rotation = headingPID.pidOut(AngleUnit.normalizeDegrees(heading - drive.getHeadingInDegrees()));

            if (timer.seconds() < 0.55) {
                drive.drive(0.5, 0, rotation);
            }
            else if (timer.seconds() < 2.5) {
                drive.drive(-0, -0.35, rotation);
            }
            else if (timer.seconds() > 4) {
                heading = -90;
                drive.drive(0,0,rotation);
            }

            telemetry.addData("hz",1/hztimer.seconds());
            hztimer.reset();
            telemetry.update();
        }
    }
}
