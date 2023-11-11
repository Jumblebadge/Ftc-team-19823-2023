package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Deposit;
import org.firstinspires.ftc.teamcode.utility.ButtonDetector;

@Config
@TeleOp(name="testing", group="Linear Opmode")
public class testing extends LinearOpMode {

    public static double Kp = 0, Kd = 0, Ki = 0, Kf = 0, Kl = 0, target = 0.5, r = 1000;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        Deposit deposit = new Deposit(hardwareMap, telemetry);

        ButtonDetector game2b = new ButtonDetector();

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
        //PhotonCore.enable();

        //Initialize FTCDashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deposit.mid();

        waitForStart();
        while (opModeIsActive()) {

            controlHub.clearBulkCache();


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

            telemetry.addData("target",target);
            telemetry.addData("dame",game2b.toggle(gamepad1.b));
            telemetry.addData("state",deposit.endPosition());

            deposit.update();
            deposit.setR(r);
            telemetry.update();
        }
    }
}
