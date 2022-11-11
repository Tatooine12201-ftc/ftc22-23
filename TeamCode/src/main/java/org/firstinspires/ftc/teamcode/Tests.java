package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp( name = "Tests")
    public class Tests extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            Mecanum mecanum = new Mecanum(hardwareMap);
            mecanum.resetEncoders();
            mecanum.setStartingPoint(0,0,0);
            double startX= mecanum.getX();
            double startY= mecanum.getY();
            double startHeading= mecanum.heading();
            double pow =0;
            waitForStart();

            telemetry.addData("y", pow);
            telemetry.update();
            mecanum.drive(0.05,0,0,false);
            String a = mecanum.toString();
            telemetry.addData("f",a);
            telemetry.update();
            sleep(2000);





        }
    }
