package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Fourbar {

    private static boolean isBusy = false;
    public ElapsedTime runtime = new ElapsedTime();
    public static boolean isReversed = false;
    public static DcMotor fourbar = null;


    LinearOpMode opMode;

    public Fourbar(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        fourbar = hw.get(DcMotor.class, "Fourbar");
        fourbar.setDirection(REVERSE);
    }

    private static final double Fourbar_speed = 1;

    public void changeDirection(boolean button) {
        if (!isBusy && !isReversed && button) {
            fourbar.setDirection(DcMotorSimple.Direction.FORWARD);
            isReversed = true;
        } else if (!isBusy && isReversed && button) {
            fourbar.setDirection(DcMotorSimple.Direction.REVERSE);
            isReversed = false;
        }
        isBusy = button;
    }


    public void spin(boolean button) {
        if( button)
        {
            fourbar.setPower(Fourbar_speed);
        }
        else{
            fourbar.setPower(0);
        }

    }
}


   // public static void spin(float left_trigger) {

       // Fourbar.setPower(Fourbar_speed);
   // }





