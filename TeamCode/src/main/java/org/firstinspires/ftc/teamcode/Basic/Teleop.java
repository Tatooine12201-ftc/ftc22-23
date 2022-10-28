package org.firstinspires.ftc.teamcode.Basic;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.Pliers;
import org.firstinspires.ftc.teamcode.lift;


@TeleOp( name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum mecanum = new Mecanum(hardwareMap);
       // lift lift = new lift(hardwareMap);
        //Pliers pliers = new Pliers(hardwareMap);
      //  pliers.close();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            mecanum.drive(gamepad1.left_stick_x,-gamepad1.left_stick_y,gamepad1.right_stick_x ,true
            );
            telemetry.addData("motors", mecanum.toString());
            telemetry.addData("gamepad x", gamepad1.left_stick_x);
            telemetry.addData("gamepad y", gamepad1.left_stick_y);
            telemetry.addData("gamepad r", gamepad1.right_stick_x);
            telemetry.update();
            //lift.up(gamepad2.dpad_up);
           // lift.down(gamepad2.dpad_down);
           // lift.move();

            //pliers.changePosition(gamepad2.y);
           // lift.changeDiraction(gamepad2.b);

           /** lift.setLevel(3);
            boolean done = lift.move();
            while (!done){
                done = lift.move();

            }
            */
        }
    }















}
