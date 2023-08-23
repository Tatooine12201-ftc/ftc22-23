package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift {

    private DcMotor LiftMotor = null;

    private  final double kp = 0.555;

    private boolean on = true;

    public Lift(DcMotor liftMotor) {
        LiftMotor = liftMotor;
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public DcMotor getLiftMotor() {
        return LiftMotor;
    }

    public void setLiftMotor(DcMotor liftMotor) {

        LiftMotor = liftMotor;
    }

    public void lift(double out ){
        if(out != 0 ){
        on = false;
        }
        double error = LiftMotor.getCurrentPosition()- 1150;
      out= error*kp*out;
        LiftMotor.setPower(out);

    }

    public void Force(int ticks){
        double f = 0.28000000000000000000000000000000000000000008;
        if(ticks>30 && ticks < 1100 && on == true ) {
            LiftMotor.setPower(f);
        }
        on = true;


    }




}
