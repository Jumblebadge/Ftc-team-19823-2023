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

@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    public static Vector2d A1 = new Vector2d(-6,0), A2 = new Vector2d(-6,4), A3 = new Vector2d(-2,4), A4 = new Vector2d(-2,0), B1 = A4, B2 = new Vector2d(-2,-4), B3 = new Vector2d(2,-4), B4 = new Vector2d(2,0), C1 = B4, C2 = new Vector2d(2,4), C3 = new Vector2d(6,4), C4 = new Vector2d(6,0);
    public static Vector2d Robot = new Vector2d(-4,4);
    public static double swingPosition = 0.5, endPosition = 0.5, servoTest = 0.5;
    FtcDashboard dashboard;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //CubicPath path = new CubicPath(telemetry,A1,A2,A3,A4,B1,B2,B3,B4,C1,C2,C3,C4);
        //GVF gvf = new GVF(dashboard,path,1);
        Deposit deposit = new Deposit(hardwareMap,telemetry);
        //ServoImplEx servo = hardwareMap.get(ServoImplEx.class,"miniD");
        //servo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        dashboard = FtcDashboard.getInstance();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        //PhotonCore.enable();

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            //telemetry.addData("gvfx",gvf.output(Robot).getX());
            //telemetry.addData("gvfy",gvf.output(Robot).getY());

            controlHub.clearBulkCache();

            deposit.setSwingPosition(swingPosition);
            deposit.setEndPosition(endPosition);
            telemetry.addData("te", swingPosition);
            //servo.setPosition(servoTest);


            telemetry.update();
        }
    }
}
