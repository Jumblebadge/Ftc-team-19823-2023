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
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Plane;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;
import org.firstinspires.ftc.teamcode.utility.CameraShenanigans;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;

@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    public static double number = 0.5, number1 = 0.5;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        CameraShenanigans camera = new CameraShenanigans(telemetry, hardwareMap, dashboard);
        Plane plane = new Plane(hardwareMap);

        ElapsedTimeW timer = new ElapsedTimeW();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        //PhotonCore.enable();

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        camera.enableAprilTag(true);
        camera.enableHSVDetection(false);
        waitForStart();
        while (opModeIsActive()) {
            plane.setPosition(number);
            camera.update();
            telemetry.addData("camera",camera.tag5Values);
            camera.telemetryAprilTag();
            telemetry.addData("pose", new Pose2d(36 + (18 - camera.tag5Values[1]),36 - camera.tag5Values[0]));
            telemetry.addData("another",gamepad2.x);
            telemetry.update();
        }
    }

}
