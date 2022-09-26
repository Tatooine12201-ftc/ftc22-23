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
            while (mecanum.getX() == startX){
                mecanum.drive(pow,0,0,false);
                sleep(1000);
                pow+=0.05;
            }
            telemetry.addData("X", pow);
            telemetry.update();

            while (mecanum.getY() == startY) {
                mecanum.drive(pow, 0, 0, false);
                sleep(1000);
                pow += 0.05;
            }
            telemetry.addData("Y", pow);
            telemetry.update();

            while (mecanum.heading() == startHeading) {
                mecanum.drive(pow, 0, 0, false);
                sleep(1000);
                pow += 0.05;
            }
            telemetry.addData("heading", pow);
            telemetry.update();

            mecanum.resetEncoders();
            mecanum.setStartingPoint(0,0,0);

            mecanum.driveTo(1000,0,0);
            mecanum.driveTo(1000,1000,0);
            mecanum.driveTo(1000,1000,90);
            mecanum.driveTo(2000,1000,90);
            mecanum.driveTo(2000,2000,90);
            mecanum.driveTo(0,0,0);
            mecanum.driveTo(0,0,270);
            mecanum.driveTo(0,0,0);
            mecanum.driveTo(1000,0,180);

        }
    }
