package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Tests")
public class Tests extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum mecanum = new Mecanum(hardwareMap, this);
        lift lift = new lift(hardwareMap, this);
        Fourbar fourbar = new Fourbar(hardwareMap, this);
        mecanum.reset();
        mecanum.setStartPos(0,0,0);

        waitForStart();

        mecanum.driveTo(0,0,90);


        // sleep(2000);
        //mecanum.driveTo(1000,0,0);


    }
}
