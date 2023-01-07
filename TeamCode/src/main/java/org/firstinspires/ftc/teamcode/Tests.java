package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Mecanum;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp( name = "Tests")
    public class Tests extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            Mecanum mecanum = new Mecanum(hardwareMap,this);
            mecanum.resetEncoders();
            mecanum.setStartingPoint(0,0,0);
            double startX= mecanum.getX();
            double startY= mecanum.getY();
            double startHeading= mecanum.heading();
            double pow =0;
            waitForStart();
            //mecanum.drive(0,0.20,0);
            //sleep(2000);
           mecanum.driveTo(1000,0, 90);
            //mecanum.driveTo(mecanum.startX,0, 0);

            telemetry.addData("motors", mecanum.toString());
            telemetry.addData("heading", mecanum.headingToDegrees());
            telemetry.addData("xl", mecanum.getXLe());
            telemetry.addData("xR", mecanum.getXRe());
            telemetry.addData("xy", mecanum.getYe());

           // sleep(2000);
           //mecanum.driveTo(1000,0,0);





        }
    }
