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

import org.firstinspires.ftc.teamcode.maths.CubicPath;
import org.firstinspires.ftc.teamcode.maths.GVF;
import org.firstinspires.ftc.teamcode.maths.PIDcontroller;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    public static final Vector2d[] points = {
            new Vector2d(-45,-60),
            new Vector2d(-45,-59),
            new Vector2d(-45,-57),
            new Vector2d(-45,-52),
            new Vector2d(-45,-47),
            new Vector2d(-45,-41),
            new Vector2d(-45,-39),
            new Vector2d(-45,-37),
            new Vector2d(-45,-36),
            new Vector2d(-45,-35.9),
            new Vector2d(-45,-35.65),
            new Vector2d(-45,-35.5)
    };
    FtcDashboard dashboard;
    private double nanoTime = 0;
    Vector2d guess;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        dashboard = FtcDashboard.getInstance();

        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, false);
        CubicPath path = new CubicPath(points);
        GVF gvf = new GVF(dashboard,path,1.6,20,1, telemetry);

        drive.setPoseEstimate(new Pose2d(-45,-60,0 ));

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        //PhotonCore.enable();

        PIDcontroller headingPID = new PIDcontroller(0.1,0.001,0,0.75,0.1);
        PIDcontroller xPID = new PIDcontroller(0.5,0,0,0.25, 0.1);
        PIDcontroller yPID = new PIDcontroller(0.5,0,0,0.25, 0.1);

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            controlHub.clearBulkCache();
            Pose2d pose = drive.getPose();
            Vector2d gvfOut = gvf.output(new Vector2d(pose.getX(), pose.getY()));

            drive.drive(gvfOut.getX(), gvfOut.getY(), gvf.headingOut(0,drive.getHeadingInDegrees(), true));

            double nano = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (nano - nanoTime));
            nanoTime = nano;

            telemetry.addData("pose",pose);
            telemetry.update();
        }
    }

}
