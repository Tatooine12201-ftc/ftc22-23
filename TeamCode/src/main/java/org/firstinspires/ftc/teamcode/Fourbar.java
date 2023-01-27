package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class Fourbar {

    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    //private static final double COUNTS_PER_RADIAN = 6.283185307179586; //
    private static final double DRIVE_GEAR_REDUCTION = 1 / 40.0 * 15 / 125;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_deg = ((COUNTS_PER_MOTOR_REV / 360) * DRIVE_GEAR_REDUCTION);
    public static boolean isReversed = false;
    public static DcMotor rightFourbar = null;
    public static DcMotor leftFourbar = null;
    private static final boolean isBusy = false;
    public ElapsedTime runtime = new ElapsedTime();
    Pid pid;
    LinearOpMode opMode;
    //    private static final double COUNTS_PER_deg = (COUNTS_PER_MOTOR_REV*DRIVE_GEAR_REDUCTION/360);
    private int level = 0;
    private final int[] levels = {0, -820, 820, 1000};
    //   private boolean fourbarIsBusy = false;
    private boolean manual = false;
    private double Fourbar_speed = 0;

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

        pid = new Pid(0.0025,0.0000000056, 0, 0, 0);
        pid.setMaxIntegral(0.065);
        pid.setTolerates(10);
        pid.DisabeleIzone();
        //pid.setF_togle(false);??   0.2
       // if (level==1){
          //  pid .setD(0.008);
        }
   // }

    public void resetEncoders() {
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


    public void setLevel(int level) {
        if (level >= 0 && level <= 3) {
            this.level = level;
        }
    }

    public boolean spin(double m) {

        double turget = levels[level];

        if (!manual) {
            if(level == 1 || level == 2){
                pid.setF_togle(true);
            }
            else {
                pid.setF_togle(false);
            }

            Fourbar_speed = pid.calculate(turget - getFourbarAngle(), System.nanoTime());
        } else {
            opMode.telemetry.addData("mn",true);
            Fourbar_speed = m;
        }

        Fourbar_speed = Range.clip(Fourbar_speed, -0.5, 0.5);


        rightFourbar.setPower(Fourbar_speed);
        leftFourbar.setPower(Fourbar_speed);
        return (Math.abs(Fourbar_speed) < 0.06);

      //  if (getFourbarAngle()>=-280 || getFourbarAngle()>=-280) {

      //  }
        };

    // }


    public double getFourbarAngle() {
        double avg = (leftFourbar.getCurrentPosition());
        return avg;
    }

    public int getEncoderRight() {
        return rightFourbar.getCurrentPosition();
    }

    public int getEncoderLeft() {
        return leftFourbar.getCurrentPosition();
    }
}








