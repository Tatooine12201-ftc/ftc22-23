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


        lift lift = new lift(hardwareMap, this);
        Pliers pliers = new Pliers(hardwareMap);
        pliers.close();
        Fourbar fourbar = new Fourbar(hardwareMap, this);

        mecanum.field = true;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            mecanum.drive(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.right_trigger - gamepad1.left_trigger, mecanum.field);
            telemetry.addData("x", -gamepad1.left_stick_y);
            telemetry.addData("y", gamepad1.right_stick_x);

            if (gamepad2.cross) {
                pliers.close();
                lift.setLevel(0);

            } else if (gamepad2.square) {
                pliers.close();
                lift.setLevel(1);

            } else if (gamepad2.triangle) {
                pliers.close();
                lift.setLevel(2);

            }

            if (gamepad2.dpad_down) {
                pliers.close();
                // lift.reset();
                fourbar.setLevel(0);
            } else if (gamepad2.dpad_left && lift.getLevel() > 0) {
                pliers.close();
                fourbar.setLevel(1);
            } else if (gamepad2.dpad_right && lift.getLevel() > 0) {
                pliers.close();
                fourbar.setLevel(2);
            } else if (gamepad2.dpad_up && lift.getLevel() > 0) {
                pliers.close();
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


            mecanum.changeMode(gamepad1.triangle);
            mecanum.setAngle(0, gamepad1.cross);
            pliers.changePosition(gamepad2.right_bumper);

            fourbar.spin(gamepad2.left_stick_x);
            lift.move(-gamepad2.right_stick_y);
            telemetry.update();
            //       fourbar.fbLeft(gamepad2.x);


        }
    }
}
