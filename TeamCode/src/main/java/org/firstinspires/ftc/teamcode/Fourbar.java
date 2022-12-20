package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Fourbar {

    private static boolean isBusy = false;
    public ElapsedTime runtime = new ElapsedTime();
    public static boolean isReversed = false;
    public static DcMotor Fourbar = null;


    LinearOpMode opMode;

    public Fourbar(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        Fourbar = hw.get(DcMotor.class, "Fourbar");
        Fourbar.setDirection(FORWARD);

    }

    private static final double Fourbar_speed = 1;

    public void changeDirection(boolean button) {
        if (!isBusy && !isReversed && button) {
            Fourbar.setDirection(DcMotorSimple.Direction.FORWARD);
            isReversed = true;
        } else if (!isBusy && isReversed && button) {
            Fourbar.setDirection(DcMotorSimple.Direction.REVERSE);
            isReversed = false;
        }
        isBusy = button;
    }


    public void spin() {
        Fourbar.setPower(1);
        isReversed = true;


        // public void stop (){
        //     Fourbar.setPower(0);
        // }


    }
}


   // public static void spin(float left_trigger) {

       // Fourbar.setPower(Fourbar_speed);
   // }





