package org.firstinspires.ftc.teamcode.Basic;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Fourbar;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.Pliers;
import org.firstinspires.ftc.teamcode.lift;


@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum mecanum = new Mecanum(hardwareMap, this);

        boolean lookR = false;
        boolean wasPresed = false;
        lift lift = new lift(hardwareMap, this);
        Pliers pliers = new Pliers(hardwareMap);
        pliers.Open();
        Fourbar fourbar = new Fourbar(hardwareMap, this);

        mecanum.field = true;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {


            if (gamepad2.dpad_down) {
//
                fourbar.setLevel(1);
            } else if (gamepad2.dpad_left) {

                fourbar.setLevel(0);
            } else if (gamepad2.dpad_right) {

                fourbar.setLevel(2);
            }

            fourbar.spin(gamepad2.left_stick_x);
        }
    }
}
