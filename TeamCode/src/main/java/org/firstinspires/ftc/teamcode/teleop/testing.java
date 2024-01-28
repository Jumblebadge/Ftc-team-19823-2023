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

    public Vector2d A1 = new Vector2d(-45,-60), A2 = new Vector2d(20,-60), A3 = new Vector2d(25,-60), A4 = new Vector2d(25,-45), B1 = A4, B2 = new Vector2d(25,-30), B3 = new Vector2d(25, -30), B4 = new Vector2d(25,-22), C1 = B4, C2 = new Vector2d(25,-14), C3 = new Vector2d(20,-12), C4 = new Vector2d(-45,-12);
    FtcDashboard dashboard;
    private double nanoTime = 0;
    Vector2d guess;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        dashboard = FtcDashboard.getInstance();

        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, false);
        CubicPath path = new CubicPath(A1,A2,A3,A4,B1,B2,B3,B4,C1,C2,C3,C4);
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

            drive.drive(gvfOut.getX(), gvfOut.getY(), gvf.headingOut(0,drive.getHeadingInDegrees()));

            double nano = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (nano - nanoTime));
            nanoTime = nano;

            telemetry.addData("pose",pose);
            telemetry.update();
        }
    }

}
