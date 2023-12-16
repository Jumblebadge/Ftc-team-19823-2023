package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.VerticalSlide;

@Config
@TeleOp(name="debugger", group="Linear Opmode")
public class debugger extends LinearOpMode {

    public static boolean activateSlides = false;
    public static double slideTarget = 0;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class that runs our linear slide
        VerticalSlide slide = new VerticalSlide(hardwareMap);
        slide.resetEncoders();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        //PhotonCore.enable();

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {

            controlHub.clearBulkCache();


            if (activateSlides) {
                slide.update();
            }


            slide.moveTo(slideTarget);
            telemetry.addData("slide",-slide.getPosition());
            telemetry.addData("slidetar",slide.getMotionTarget());
            telemetry.update();
        }
    }
}
