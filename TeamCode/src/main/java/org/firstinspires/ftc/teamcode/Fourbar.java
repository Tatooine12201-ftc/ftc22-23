package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class Fourbar {

    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder

    private static final double DRIVE_GEAR_REDUCTION =  56.0/30.0;     // This is < 1.0 if geared UP
    private static final double tiksPerDegree = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/360;

    public static DcMotor rightFourbar = null;
    public static DcMotor leftFourbar = null;
    private static final boolean isBusy = false;
    ElapsedTime time =new ElapsedTime();
    Pid pid;
    LinearOpMode opMode;

    private int level = 0;
    private final int[] levels = {0,138,-138,180} ;//140
    private boolean manual = false;
    private double Fourbar_speed = 0;
    int prevLevel = 0;
    double F =0;

    public Fourbar(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        rightFourbar = hw.get(DcMotor.class, "rightFourbar");
        rightFourbar.setDirection(REVERSE);

        leftFourbar = hw.get(DcMotor.class, "leftFourbar");
        leftFourbar.setDirection(REVERSE);


        leftFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new Pid( 0.016953,0.0001, 0, 0.08, true);
        F = pid.getF();
        pid.setMaxIntegral(0.1);
        pid.setTolerates(3);


        }







    public void resetEncoders()
    {
        leftFourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        manual = false;
        level = 0;
    }
    public void reset() {
        resetEncoders();
    }




    public void setManual() {
        manual = true;
    }


    public void setLevel(int level)
    {
        if (level >= 0 && level <= 3)
        {
            this.level = level;
        }
    }
    public boolean spinTo(int level) {
        //the default timeout is 5 seconds
        return spinTo(level, 3000);
    }
    public boolean spinTo(int level, double timeOut) {
        boolean done = false;
        double startTime = time.milliseconds();
        do {

            //check if the robot has tried for more than timeOut milliseconds
            if (time.milliseconds() - startTime > timeOut ) {
               rightFourbar.setPower(0);
               leftFourbar.setPower(0);
                //return false
                opMode.telemetry.clearAll();
                opMode.telemetry.addData("timeOut", "timeOut");

                return false;
            }
            setLevel(level);
            pid.setF(F);
            done = spin(0);
        }while (!done);
        pid.setF(F);
        spin(0,true);
        return true;
    }
    public boolean spin(double m){
        return spin(m,false);
    }
    public boolean spin(double m, boolean f) {
        double turget = levels[level];
        double err = turget - getFourbarAngle();
        if (opMode.opModeIsActive() && !opMode.isStopRequested()) {


            if (!manual) {
                if (level == 1 || level == 2) {
                    if (err >0 || f){
                        pid.setF(F);
                    }
                    else {
                        pid.setF(0);
                    }
                    pid.setF_togle(true);
                } else {
                    pid.setF_togle(false);
                }

                Fourbar_speed = pid.calculate(err);
            } else {
                opMode.telemetry.addData("mn", true);
                Fourbar_speed = m;
            }

            Fourbar_speed = Range.clip(Fourbar_speed, -0.6, 0.6);
            prevLevel = level;


            opMode.telemetry.addData("val", Fourbar_speed);
            opMode.telemetry.addData("test", getFourbarAngle());
            opMode.telemetry.addData("ticks fl", leftFourbar.getCurrentPosition());
            opMode.telemetry.addData("ticks fr", rightFourbar.getCurrentPosition());
            rightFourbar.setPower(Fourbar_speed);
            leftFourbar.setPower(Fourbar_speed);
        }
        else if (opMode.opModeInInit()){
            rightFourbar.setPower(0);
            leftFourbar.setPower(0);
        }
        return (Math.abs(Fourbar_speed) < 0.1);


        }



    public double getFourbarAngle() {

        double avg = ( rightFourbar.getCurrentPosition());
        return avg /tiksPerDegree;

    }

    public int getEncoderRight() {
        return rightFourbar.getCurrentPosition();
    }

    public int getEncoderLeft() {
        return leftFourbar.getCurrentPosition();
    }
}







