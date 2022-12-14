package org.firstinspires.ftc.teamcode;

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
    public static DcMotor rightFourbar = null;
    public static DcMotor leftFourbar = null;
    Pid pid ;
    private static final double COUNTS_PER_MOTOR_REV = 1120 ;    // eg: TETRIX Motor Encoder
    //private static final double COUNTS_PER_RADIAN = 6.283185307179586; //
    private static final double DRIVE_GEAR_REDUCTION = 1/40.0 * 15/125;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_deg = ((COUNTS_PER_MOTOR_REV /360)* DRIVE_GEAR_REDUCTION);
//    private static final double COUNTS_PER_deg = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION/360);
    private int level = 1;
    private int[] levels = {-30,0,30};
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

        leftFourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        pid = new Pid(0.162,0.0015,0.000015,0);
        pid.setMaxIntegral(0.1);
        pid.setTolerates(0.8);
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
        if (level >= 0 && level <= 2){
            this.level = level;
        }
    }


    public void spin(double m) {
        opMode.telemetry.addData("a",getFourbarAngle());
        opMode.telemetry.addData("m", manual);
        opMode.telemetry.addData("fs", Fourbar_speed);
        double a = levels[level];
        opMode.telemetry.addData("a", a);

        if (!manual){

            Fourbar_speed = pid.calculate(a - getFourbarAngle());
        }
        else{
            Fourbar_speed = m;
        }




        rightFourbar.setPower(Fourbar_speed);
        leftFourbar.setPower(Fourbar_speed);

    }


    public double getFourbarAngle(){
        double avg = (rightFourbar.getCurrentPosition());
        return avg * COUNTS_PER_deg;
    }
}








