package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.awt.font.NumericShaper;


public class Fourbar {

    private static final double COUNTS_PER_MOTOR_REV = 560;    // eg: TETRIX Motor Encoder

    private static final double DRIVE_GEAR_REDUCTION = 52.0 / 30.0;     // This is < 1.0 if geared UP
    private static final double tiksPerDegree = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;

    private static final double SERVO_DEGREE_TO_FOURBAR_DEGREE = DRIVE_GEAR_REDUCTION * 360;

    public DcMotorEx Fourbar = null;

    private static final boolean isBusy = false;
    Pid pid;
    LinearOpMode opMode;
    private int level = 0;
    private final int[] levels = {0,138,-138,180,143,-143} ;//140

    private boolean manual = false;


    private double Fourbar_speed = 0;

    int prevLevel = 0;
    double F = 0;

    public Fourbar(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        Fourbar = hw.get(DcMotorEx.class, "Fourbar");
        Fourbar.setDirection(REVERSE);



        Fourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      //  pid = new Pid( 0.02 ,0.0005, 0.0001, 0.0006);
        pid = new Pid( 0.075 ,0.005, 0.001,  0.0001);

        // 0.016953
        F = pid.getF();
        pid.setIntegrationBounds(-0.1,0.1);
        pid.setTolerance(1);





    }


    public void resetEncoders() {
        Fourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Fourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        manual = false;
        level = 0;
    }

    private void stopm() {
        Fourbar.setPower(0);


    }

    public void reset() {
        resetEncoders();
    }

    public boolean isatpos (){
        return (pid.atSetPoint());
    }


    public void setManual() {
        manual = true;
    }

    public void setLevel(int level) {
        if (level >= 0 && level <= 5) {
            this.level = level;
        }
    }


    public boolean spin(double m) {
        double turget = levels[level];
        double out ;
        if (!manual){
            out = pid.calculate(getEncoder(), turget);
        }
        else {
            out=m ;
        }

       // else if (opMode.opModeInInit()){
          // Fourbar.setPower(0);

       // }
       // if (level == 0){
           // pid.setTolerance(5);
            //if (pid.atSetPoint()){
           // out =0;}
           // Fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      //  }
        //else {
          //  pid.setTolerance(1.2);
           // Fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // }

        if (out > 0.6){
            out = 0.6;
        }
        else if (out < -0.6){
            out = -0.6;

        }



        Fourbar.setPower(out);


        return (pid.atSetPoint());
    }

    public double getEncoder() {
        return  Fourbar.getCurrentPosition() / tiksPerDegree;
    }
}









