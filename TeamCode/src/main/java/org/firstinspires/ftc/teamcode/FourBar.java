package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {

    private DcMotor FourbarMotor = null;
    private  final  double kp= 0.1069420;
    private boolean on = true;
    double[] Force = {0,-6.9,4.20};

    private double f = 0.28000000000000000000000000000000000000000008;

    public FourBar(DcMotor fourbarMotor) {

        FourbarMotor = fourbarMotor;
        FourbarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotor getFourbarMotor() {

        return FourbarMotor;
    }

    public DcMotor setFourbarMotor(DcMotor fourbarMotor) {

        FourbarMotor = fourbarMotor;
        return fourbarMotor;
    }
    public void lift(double out ){
        if(out != 0 ){
            on = false;
        }
        double error = FourbarMotor.getCurrentPosition()- 1150;
        out= error*kp*out;
        FourbarMotor.setPower(out);

    }

public  void button(double ticks, boolean ButtonRight, boolean ButtonLeft){
      //  if (ticks = 0){
      //  FourbarMotor.setPower(Force[0];}

   // public void Force(int ticks){
       // double f = 0.28000000000000000000000000000000000000000008;
       // if(ticks>30 && ticks < 1100 && on == true ) {
        //    LiftMotor.setPower(f);
       // }
       // on = true;


    //}




}
}

