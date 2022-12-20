package org.firstinspires.ftc.teamcode.Basic;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Fourbar;
import org.firstinspires.ftc.teamcode.Mecanum;
import org.firstinspires.ftc.teamcode.Pliers;
import org.firstinspires.ftc.teamcode.lift;



@TeleOp( name = "Teleop")
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum mecanum = new Mecanum(hardwareMap, this);
        mecanum.resetEncoders();

        lift lift = new lift(hardwareMap, this);
        Pliers pliers = new Pliers(hardwareMap);
        pliers.close();
        Fourbar fourbar = new Fourbar(hardwareMap, this);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            mecanum.drive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, true
            );
            telemetry.addData("motors", mecanum.toString());
            telemetry.addData("heding", mecanum.headingToDegrees());
            telemetry.addData("xl", mecanum.getXLe());
            telemetry.addData("xR", mecanum.getXRe());
            telemetry.addData("xy", mecanum.getYe());
            telemetry.addData("gamepad x", gamepad1.left_stick_x);
            telemetry.addData("gamepad y", gamepad1.left_stick_y);
            telemetry.addData("gamepad r", gamepad1.right_stick_x);
            telemetry.update();

            lift.up(gamepad2.dpad_up);
            lift.down(gamepad2.dpad_down);
            lift.move();

            pliers.changePosition(gamepad2.y);
            fourbar.changeDirection(gamepad2.left_stick_button);
            fourbar.spin(gamepad2.x);
        }
    }
}
