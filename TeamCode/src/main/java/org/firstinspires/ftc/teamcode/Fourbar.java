package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class Fourbar {

    private static boolean isBusy = false;
    public ElapsedTime runtime = new ElapsedTime();
    public static boolean isReversed = false;
    public static DcMotor rightFourbar = null;
    public static DcMotor leftFourbar = null;
    Pid pid ;
    private static final double COUNTS_PER_MOTOR_REV = 1120 ;    // eg: TETRIX Motor Encoder
    //private static final double COUNTS_PER_RADIAN = 6.283185307179586; //
    private static final double DRIVE_GEAR_REDUCTION = 1/40.0 * 15/125;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_deg = ((COUNTS_PER_MOTOR_REV /360)* DRIVE_GEAR_REDUCTION);
//    private static final double COUNTS_PER_deg = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION/360);
    private int level = 0;
    private int[] levels = {0,-820,820,1000};
    private boolean manual = false;
 //   private boolean fourbarIsBusy = false;

    LinearOpMode opMode;

    public Fourbar(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        rightFourbar = hw.get(DcMotor.class, "rightFourbar");
        rightFourbar.setDirection(REVERSE);

        leftFourbar= hw.get(DcMotor.class, "leftFourbar");
        leftFourbar.setDirection(REVERSE);



        leftFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new Pid(0.0015,0.000006,0.002,0);
        pid.setMaxIntegral(0.065);
        pid.setTolerates(30);
    }

  private double Fourbar_speed = 0;



    public void reset(){
        leftFourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        manual = false;
        level = 1;
    }


    public void setManual(){
        manual = true;
    }


    public void setLevel(int level){
        if (level >= 0 && level <= 3){
            this.level = level;
        }
    }


    public void spin(double m) {
        opMode.telemetry.addData("m", manual);
       opMode.telemetry.addData("4bartiks", Fourbar.rightFourbar.getCurrentPosition());
        opMode.telemetry.addData("4bartiksL", Fourbar.leftFourbar.getCurrentPosition());
        opMode.telemetry.addData("Fourbar_speed", Fourbar_speed);
        opMode.telemetry.addData("level", level);
        opMode.telemetry.addData("m",m);

        double turget = levels[level];
        opMode.telemetry.addData("turget", turget);

        if ( !manual ){

            Fourbar_speed = pid.calculate(turget - getFourbarAngle());
            opMode.telemetry.addData("err",turget - getFourbarAngle());
        }
        else{
            Fourbar_speed = m;
        }

        Fourbar_speed = Range.clip(Fourbar_speed,-0.5,0.5);


        rightFourbar.setPower(Fourbar_speed);
        leftFourbar.setPower(Fourbar_speed);

    }


    public double getFourbarAngle(){
        double avg = (leftFourbar.getCurrentPosition());
        return avg;
    }
    public int getEncoderRight(){
        return rightFourbar.getCurrentPosition();
    }
    public int getEncoderLeft(){
        return leftFourbar.getCurrentPosition();
    }
}








