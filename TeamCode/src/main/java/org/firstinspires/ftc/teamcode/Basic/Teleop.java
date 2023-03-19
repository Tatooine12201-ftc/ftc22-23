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
        fourbar.setLevel(0);
        lift.setLevel(2);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            fourbar.spin(gamepad2.left_stick_x);
            lift.move(gamepad2.right_stick_y);
            if (gamepad2.cross){
                fourbar.setLevel(2);
            }
            else if (gamepad2.circle){
                fourbar.setLevel(0);
            }
            else if (gamepad2.triangle){
                fourbar.setLevel(1);
            }
            telemetry.update();



            if (gamepad2.share) {
            fourbar.setManual();
            lift.setManual();
        }
        if (gamepad2.options) {
            fourbar.reset();
            lift.reset();
        }


        }
    }
}
