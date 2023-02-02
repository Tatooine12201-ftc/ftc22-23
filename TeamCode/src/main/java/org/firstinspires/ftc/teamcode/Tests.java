package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
        telemetry.clearAll();

        waitForStart();
      mecanum.driveTo(1000,0,0,300000);
      telemetry.addData("r",mecanum.Heading());
      sleep(200);

    }
}
