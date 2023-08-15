package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {

    private DcMotor LiftMotor = null;

    private  final double kp =0.555;

    public Lift(DcMotor liftMotor) {
        LiftMotor = liftMotor;
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotor getLiftMotor() {
        return LiftMotor;
    }

    public void setLiftMotor(DcMotor liftMotor) {
        LiftMotor = liftMotor;
    }

    public void lift(double out ){
      out= error*kp*out;
        LiftMotor.setPower(out);

    }
    public void Force(int ticks){
        double f = 0.12;
        if(ticks>30 && ticks < 1100) {
            LiftMotor.setPower(f);
        }
    }
    double error = LiftMotor.getCurrentPosition()- 1150;




}
