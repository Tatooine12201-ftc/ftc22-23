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
        CompletableFuture<Void> drive_thread = null;

        mecanum.reset();
        mecanum.setStartPos(0,0,0);

        waitForStart();

        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.N) {
            drive_thread = mecanum.driveTo(500,500,90);
            sleep(200);

        }
        while(!drive_thread.isDone());


    }
}
