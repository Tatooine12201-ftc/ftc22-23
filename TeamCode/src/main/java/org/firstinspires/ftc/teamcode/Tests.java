package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;
import org.firstinspires.ftc.teamcode.Fourbar;
import org.firstinspires.ftc.teamcode.lift;

import java.util.concurrent.CompletableFuture;


@TeleOp(name = "Tests")
public class Tests extends ThreadOpMode {
    Fourbar fourbar;
    Mecanum mecanum ;
    lift lift;
    @Override
    public void mainInit() {
        fourbar = new Fourbar(hardwareMap, this);
        mecanum = new Mecanum(hardwareMap, this);
        lift= new lift(hardwareMap, this);
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void subsystemLoop() {
                //fourbar.spin(0);
            }
        }));



    }
    @Override
    public void mainLoop() {

    }

    @Override
    public void runOpMode() throws InterruptedException {
        mainInit();
        waitForStart();
        startThreads();

      // mecanum.driveTo(0,0,0);


        while (opModeIsActive() && !isStopRequested()) {

            lift.move(0);
            lift.setLevel(2);
            fourbar.setLevel(1);
            fourbar.spin(0);

            telemetry.addData("fe " , fourbar.getEncoder());


            telemetry.update();

        }


    }




}
