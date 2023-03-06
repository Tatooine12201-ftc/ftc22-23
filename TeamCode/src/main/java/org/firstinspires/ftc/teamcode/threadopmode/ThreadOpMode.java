package org.firstinspires.ftc.teamcode.threadopmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.List;

/**riod
 * A type of {} that contains threads to be ran in parallel peically.
 * Register threads with {@link ThreadOpMode#registerThread(TaskThread)}
 */
public abstract class ThreadOpMode extends LinearOpMode {
    private List<TaskThread> threads = new ArrayList<>();

    /**
     * Registers a new {@link TaskThread} to be ran periodically.
     * Registered threads will automatically be started during
     *
     * @param taskThread A {@link TaskThread} object to be ran periodically.
     */
    public final void registerThread(TaskThread taskThread) {
        threads.add(taskThread);
    }
    public abstract void mainInit();
    public abstract void mainLoop();


    public final void startThreads() {
        for(TaskThread taskThread : threads) {
            taskThread.start();
        }
    }


    public final void stopThreads() {
        for(TaskThread taskThread : threads) {
            taskThread.stop();
        }
    }
}
