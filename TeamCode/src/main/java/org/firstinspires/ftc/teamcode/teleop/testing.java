package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
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
import org.firstinspires.ftc.teamcode.utility.ElapsedTimeW;

@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    public Vector2d A1 = new Vector2d(-72,0), A2 = new Vector2d(-72,30), A3 = new Vector2d(-30,30), A4 = new Vector2d(-30,0), B1 = A4, B2 = new Vector2d(-30,-30), B3 = new Vector2d(30, -30), B4 = new Vector2d(30,0), C1 = B4, C2 = new Vector2d(30,30), C3 = new Vector2d(72,30), C4 = new Vector2d(72,0);
    public static double x = -4, y = 4;
    public Vector2d Robot = new Vector2d(x,y);
    FtcDashboard dashboard;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        dashboard = FtcDashboard.getInstance();

        CubicPath path = new CubicPath(telemetry,A1,A2,A3,A4,B1,B2,B3,B4,C1,C2,C3,C4);
        GVF gvf = new GVF(dashboard,path,1);

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
            Robot = new Vector2d(x, y);
            Vector2d gvfOut = gvf.output(Robot);

            path.setControlPoints(A1, A2, A3, A4, B1, B2, B3, B4, C1, C2, C3, C4);

            telemetry.addData("gvfx",gvfOut.getX());
            telemetry.addData("gvfy",gvfOut.getY());

            //servo.setPosition(servoTest);


            telemetry.update();
        }
    }
}
