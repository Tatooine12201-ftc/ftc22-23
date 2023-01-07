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
        mecanum.setStartingPoint(0,0,0);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            mecanum.drive(gamepad1.right_stick_x,-gamepad1.left_stick_y, gamepad1.right_trigger-gamepad1.left_trigger);
//            telemetry.addData("motors", mecanum.toString());
//            telemetry.addData("heding", mecanum.headingToDegrees());
//            telemetry.addData("xl", mecanum.getXLe());
//            telemetry.addData("xR", mecanum.getXRe());
//            telemetry.addData("xy", mecanum.getYe());
//            telemetry.addData("gamepad x", gamepad1.left_stick_x);
//            telemetry.addData("gamepad y", gamepad1.left_stick_y);
//            telemetry.addData("gamepad r", gamepad1.right_stick_x);


          //  lift.up(gamepad2.dpad_up);
         //   lift.down(gamepad2.dpad_down);
           if (gamepad2.cross){
               pliers.close();
               lift.setLevel(0);

           }
           else if(gamepad2.square){
               pliers.close();
               lift.setLevel(1);

           }
           else if (gamepad2.circle){
               pliers.close();
               lift.setLevel(2);

           }
           else if (gamepad2.triangle)
           {
               pliers.close();
               lift.setLevel(3);

           }
            lift.move();

            if(gamepad1.cross){
                mecanum.resetEncoders();
            }
            if (gamepad2.dpad_down){
                pliers.close();
                // lift.reset();
                fourbar.setLevel(1);
            }
            else if(gamepad2.dpad_left && lift.getLevel()>0)
            {
                fourbar.setLevel(0);
            }
            else if(gamepad2.dpad_right && lift.getLevel() >0 ){
                fourbar.setLevel(2);
            }

            if(gamepad2.options)
            {
                fourbar.reset();
            }
            if (gamepad2.share){
                fourbar.setManual();
            }

          //  if (gamepad1.x){
            //    mecanum.
           // }

                mecanum.changePosition(gamepad1.triangle);
            if (gamepad1.back ){


            }



            pliers.changePosition(gamepad2.right_bumper);
            fourbar.spin(gamepad2.left_stick_x);
            telemetry.update();
     //       fourbar.fbLeft(gamepad2.x);
            telemetry.addData("xl", mecanum.getXLe());
            telemetry.addData("xR", mecanum.getXRe());
            telemetry.addData("xy", mecanum.getYe());



        }
    }
}
