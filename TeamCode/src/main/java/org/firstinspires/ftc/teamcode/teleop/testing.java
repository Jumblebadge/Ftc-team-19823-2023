package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.maths.CubicPath;
import org.firstinspires.ftc.teamcode.maths.GVF;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;

@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {


    public static boolean endIn = true;
    public static double dTime = 1;
    double count = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        Deposit deposit = new Deposit(hardwareMap);
        ButtonDetector depositLatch = new ButtonDetector();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        //PhotonCore.enable();

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Gamepad current = new Gamepad();
        Gamepad previous = new Gamepad();


        waitForStart();
        while (opModeIsActive()) {
            previous.copy(current);
            current.copy(gamepad2);
            controlHub.clearBulkCache();


            deposit.toggleLatch(current.x && !previous.x, dTime);
            if (depositLatch.risingEdge(gamepad2.x)) count++;
            deposit.update(endIn);
            deposit.in();
            telemetry.addData("sllide",count);
            telemetry.addData("someghind",current.x && !previous.x);
            telemetry.addData("another",gamepad2.x);
            telemetry.update();
        }
    }

}
