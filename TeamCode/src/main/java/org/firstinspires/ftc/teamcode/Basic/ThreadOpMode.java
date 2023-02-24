package org.firstinspires.ftc.teamcode.Basic;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Fourbar;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.Pliers;
import org.firstinspires.ftc.teamcode.lift;
import java.util.ArrayList;
import java.util.List;

/**
 * A type of {@link LinearOpMode } that contains threads to be ran in parallel periodically.
 * Register threads with {@link ThreadOpMode#registerThread(TaskThread)}
 */

public abstract class ThreadOpMode extends LinearOpMode {
    private List<TaskThread> threads = new ArrayList<>();


    public void runOpMode() throws InterruptedException {
        //threads

        /*
        registerThreads and stuff

         */

        Thread mecanum_thread = new Thread();


        TaskThread lift_thread = new TaskThread();
        registerThread( lift_thread);




        start_threads();
        //init stuff
        Mecanum mecanum = new Mecanum(hardwareMap, this);

        boolean lookR = false;
        boolean wasPresed = false;
        lift lift = new lift(hardwareMap, this);
        Pliers pliers = new Pliers(hardwareMap);
        pliers.Open();
        Fourbar fourbar = new Fourbar(hardwareMap, this);

        mecanum.field = true;

        waitForStart();
        while (opModeIsActive()&& !isStopRequested()){

        }
        stop_threads();
    }



    /**
     * Registers a new {@link TaskThread} to be ran periodically.
     * Registered threads will automatically be started during {@link LinearOpMode#start()} and stopped during {@link LinearOpMode #stop()}.
     *
     * @param taskThread A {@link TaskThread} object to be ran periodically.
     */
    public final void registerThread(TaskThread taskThread) {
        threads.add(taskThread);
    }

    /**
     * Should not be called by subclass.
     */
    public final void start_threads() {
        for(TaskThread taskThread : threads) {
            taskThread.start();
        }
    }

    /**
     * Should not be called by subclass.
     */
    public final void stop_threads() {
        for(TaskThread taskThread : threads) {
            taskThread.stop();
        }
    }
}