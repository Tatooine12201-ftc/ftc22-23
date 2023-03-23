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
        lift lift = new lift(hardwareMap, this);
       Fourbar fourbar = new Fourbar(hardwareMap, this);

        mecanum.reset();
        mecanum.setStartPos(0,0,0);


        waitForStart();

        //sleep(1000);
//1000
     /// mecanum.driveTo(500,500,90);
      mecanum.driveTo(1200,0,0);
     // mecanum.driveTo(0,00,0);
     // telemetry.addData("rr",Mecanum.)
      //sleep(100000000);
        //mecanum.driveTo(0,0,90,10000000);

    }
}
