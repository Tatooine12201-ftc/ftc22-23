package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {

    private DcMotor FourbarMotor = null;
    private  final  double kp= 0.1069420;
    private  final double ki = 0.01;
    private  final double kd = 0.05;
    private boolean on = true;
    double[] Force = {0,-6.9,4.20};
    int[] levels = {0,255,-255};
    private final double tickPerRotaion = 560;
    private  final  double ratio = 52.0/30.0;
    private final  double ticksPerDeg = tickPerRotaion*ratio/360;
    double shoom = 0;
    double error = 0;
    double prvError = 0;
    double there = 0;
    double out2 = 0;


 public int TicksPerDeg( double deg){
    int ticks1 = ticksPerDeg*deg;
    return ticks1
 }

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
    public void spin(int pos ){

        error = FourbarMotor.getCurrentPosition()- TicksPerDeg(levels[pos]) ;
        there = error-prvError;
        prvError = error;
        shoom += error;
        out2= error*kp+ki*shoom+there*kd;
        FourbarMotor.setPower(out2);

    }

public  void button(double pos){


       FourbarMotor.setPower(Force[pos]);





}
}

