package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mecanum;

import java.util.concurrent.CompletableFuture;


@TeleOp(name = "Tests")
public class Tests extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum mecanum = new Mecanum(hardwareMap, this);
        //lift lift = new lift(hardwareMap, this);
       // Fourbar fourbar = new Fourbar(hardwareMap, this);

        mecanum.reset();
        mecanum.setStartPos(0,0,0);


        waitForStart();
       // mecanum.drive(0.3,0,0,false,false);
        //sleep(1000);

      mecanum.driveTo(50,0,0,1000);
        mecanum.driveTo(50,0,90,1000);
    }
}
