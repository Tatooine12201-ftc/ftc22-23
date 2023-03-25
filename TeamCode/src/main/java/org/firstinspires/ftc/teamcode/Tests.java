package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

import java.util.concurrent.CompletableFuture;


@TeleOp(name = "Tests")
public class Tests extends ThreadOpMode {
    Fourbar fourbar;
    Mecanum mecanum ;
    @Override
    public void mainInit() {
        //fourbar = new Fourbar(hardwareMap, this);
        mecanum = new Mecanum(hardwareMap, this);



    }

    @Override
    public void runOpMode() throws InterruptedException {
        mainInit();
        waitForStart();
        startThreads();
       mecanum.driveTo(500,500,0);

    }



    @Override
    public void mainLoop() {

    }
}
