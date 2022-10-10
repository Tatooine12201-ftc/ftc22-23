package org.firstinspires.ftc.teamcode.Basic;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanum;



@TeleOp( name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum mecanum = new Mecanum(hardwareMap);
        // mecanum.resetEncoders();
        //  mecanum.setStartingPoint(0,0,0);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            mecanum.drive(
                    gamepad1.left_stick_x,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    true
            );
        //    telemetry.addData("tickes" , mecanum.getXTicks());
         //   telemetry.addData("x" , mecanum.getX());
           // telemetry.update();
           // sleep(50);

        }
    }















}
