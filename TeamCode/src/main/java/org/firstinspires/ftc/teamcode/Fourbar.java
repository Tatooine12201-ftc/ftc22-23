package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public DcMotor Fourbar = null;

    private static final boolean isBusy = false;
    Pid pid;
    LinearOpMode opMode;
    private int level = 0;
    private final int[] levels = {0,138,-138,180} ;//140

    private boolean manual = false;


    private double Fourbar_speed = 0;

    int prevLevel = 0;
    double F = 0;

    public Fourbar(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        Fourbar = hw.get(DcMotor.class, "Fourbar");
        Fourbar.setDirection(REVERSE);


        Fourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new Pid( 0.007 ,0.0001, 0, 0);

        // 0.016953
        F = pid.getF();
        pid.setIntegrationBounds(-0.1,0.1);
        pid.setTolerance(0);
        stopm();




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


    public void setManual() {
        manual = true;
    }

    public void setLevel(int level) {
        if (level >= 0 && level <= 3) {
            this.level = level;
        }
    }


    public boolean spin(double m) {
        double turget = levels[level];
        double out ;
        if (!manual){
            out= pid.calculate(getEncoder(), turget);
        }
        else {
            out=m ;
        }

       // else if (opMode.opModeInInit()){
          //  Fourbar.setPower(0);

       // }
        out= Range.clip(out,-0.7,0.8);
        Fourbar.setPower(out);


        opMode.telemetry.addData("tur", turget);
        opMode.telemetry.addData("ticks", getEncoder());




        return (pid.atSetPoint());






    }

    public double getEncoder() {
        return  Fourbar.getCurrentPosition() / tiksPerDegree;
    }
}









