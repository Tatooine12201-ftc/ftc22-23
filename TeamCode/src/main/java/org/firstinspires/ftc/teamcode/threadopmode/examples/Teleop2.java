package org.firstinspires.ftc.teamcode.threadopmode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Fourbar;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.Pliers;
import org.firstinspires.ftc.teamcode.lift;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;

//Extend ThreadOpMode rather than OpMode
@TeleOp(name = "Teleop2")

public class Teleop2 extends ThreadOpMode {

    Mecanum mecanum;
    boolean lookR = false;
    boolean wasPresed = false;
    lift lift;
    Fourbar fourbar;
    Pliers pliers;
    //Define global variables
    DcMotor a;
    DcMotor b;

    boolean liftDone;

    @Override
    public void mainInit() {
        mecanum = new Mecanum(hardwareMap, this);
         lookR = false;
         wasPresed = false;
        lift = new lift(hardwareMap, this);
        pliers = new Pliers(hardwareMap);
        pliers.Open();
        fourbar = new Fourbar(hardwareMap, this);
        mecanum.field = true;
        mecanum.setStartPos(0,0,90);

        //lift
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void subsystemLoop() {
                fourbar.spin(gamepad2.left_stick_x);
            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void subsystemLoop() {
                lift.move(-gamepad2.right_stick_y);
            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void subsystemLoop() {
                mecanum.drive(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.right_trigger - gamepad1.left_trigger, mecanum.field, lookR);
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

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.circle && !wasPresed) {
                lookR = !lookR;
                if (lookR) {
                    mecanum.setWantedAngle();
                }
                wasPresed = true;
            } else if (!gamepad1.circle) {
                wasPresed = false;
            }

            if (gamepad2.cross) {
               pliers.Open();
                lift.setLevel(0);
            }else if (gamepad2.circle) {
                pliers.Open();
                lift.setLevel(1);
            }
            else if (gamepad2.square) {
                pliers.Open();
                lift.setLevel(2);
            } else if (gamepad2.triangle) {
                pliers.Open();
                lift.setLevel(3);
            }

            if (gamepad2.dpad_down) {
                pliers.Open();
                fourbar.setLevel(0);
            } else if (gamepad2.dpad_left && lift.getLevel() > 0) {
                pliers.Open();
                fourbar.setLevel(2);
            } else if (gamepad2.dpad_right && lift.getLevel() > 0) {
                pliers.Open();
                fourbar.setLevel(1);
            } else if (gamepad2.dpad_up && lift.getLevel() > 0) {
                pliers.Open();
                fourbar.setLevel(3);
            }

            if (gamepad2.options) {
                fourbar.reset();
                lift.reset();
            }
            if (gamepad2.share) {
                fourbar.setManual();
                lift.setManual();
            }
            pliers.changePosition(gamepad2.right_bumper);

            mecanum.changeMode(gamepad1.triangle);
            mecanum.setAngle(0, gamepad1.cross);
            telemetry.addData("fe " , fourbar.getEncoder());



            telemetry.update();




        }
        stopThreads();

    }
}
