package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.maths.CubicPath;
import org.firstinspires.ftc.teamcode.maths.GVF;
import org.firstinspires.ftc.teamcode.maths.GoToPoint;
import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;

@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    public Vector2d A1 = new Vector2d(-72,0), A2 = new Vector2d(-72,30), A3 = new Vector2d(-30,30), A4 = new Vector2d(-30,0), B1 = A4, B2 = new Vector2d(-30,-30), B3 = new Vector2d(30, -30), B4 = new Vector2d(30,0), C1 = B4, C2 = new Vector2d(30,30), C3 = new Vector2d(72,30), C4 = new Vector2d(72,0);
    public static double x = -4, y = 4;
    public Vector2d Robot = new Vector2d(x,y);
    FtcDashboard dashboard;
    private double nanoTime = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        dashboard = FtcDashboard.getInstance();

        MecanumDrive drive = new MecanumDrive(telemetry, hardwareMap, false);
        CubicPath path = new CubicPath(telemetry,A1,A2,A3,A4,B1,B2,B3,B4,C1,C2,C3,C4);
        GVF gvf = new GVF(dashboard,path,1, telemetry);

        ElapsedTimeW timer = new ElapsedTimeW();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        //PhotonCore.enable();

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            controlHub.clearBulkCache();

            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();
            Pose2d pose = drive.getPose();
            drawRobot(fieldOverlay, pose);
            packet.put("x", pose.getX());
            packet.put("y", pose.getY());
            packet.put("heading (deg)", Math.toDegrees(pose.getHeading()));


            Vector2d gvfOut = gvf.output(new Vector2d(pose.getX(), pose.getY()));


            path.setControlPoints(A1, A2, A3, A4, B1, B2, B3, B4, C1, C2, C3, C4);

            double nano = System.nanoTime();
            telemetry.addData("hz", 1000000000 / (nano - nanoTime));
            nanoTime = nano;

            telemetry.addData("gvfx",gvfOut.getX());
            telemetry.addData("gvfy",gvfOut.getY());
            telemetry.addData("pose",pose);

            //servo.setPosition(servoTest);


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
