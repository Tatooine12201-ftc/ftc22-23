package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.HardwareMap.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class lift  {
    //Thread lift_thread = new Thread();
    private final boolean liftIsBusy = false;

    private static double GEAR_RATIO = 1/2;
    public static double TICKS_PER_REV = 8192;
    public static double PULY_PERIMITAR = 136.85;
    private static final double COUNTS_PER_MM = (TICKS_PER_REV * GEAR_RATIO) / PULY_PERIMITAR;


    private final int[] levels = {
            10,//0
            100,//1
            840,//2
            1300,//3
            850,//4
            560,//5
            800,//6
            400,//7
            200,//8
            450 ,//9

    };

    private final boolean isBusy = false;
    private final boolean isBusy2 = false;
    private final boolean isBusy3 = false;
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor lift = null;
    public DcMotor liftTwo = null;
    int autoHige = 4;
    int autoStack4 = 5;
    int liftStack = 6;
    int autoStack3 = 9;
    int abouveWheel = 7;
    LinearOpMode opMode;

    int prevLevel = 0;
    double F = 0;

    private int level = 0;
    private boolean manual = false;
    private Pid pid;


    public lift(HardwareMap hw, LinearOpMode opMode) {

        this.opMode = opMode;

        lift = hw.get(DcMotor.class, "lift");
        lift.setDirection(REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        resetEncoders();

        pid = new Pid(0.005, 0.0001, 0, 0.19);

        F = pid.getF();

        pid.setIntegrationBounds(0,0.27);
        pid.setTolerance(10);


        stopm();

    }

    private void stopm() {
        lift.setPower(0);


    }


    public lift(DcMotor lift) {
        this.lift = lift;
    }

    public int getLevel() {
        return this.level;
    }

    public void setLevel(int level) {
        if (level >= 0 && level <= 7) {
            this.level = level;
        }
    }

    public int getEncoder() {
        return lift.getCurrentPosition();
    }



    public boolean cheklevel (){
        boolean isFinnished  = false;
        if ((liftTwo.getCurrentPosition() <= levels[level] + 3 && liftTwo.getCurrentPosition() >= levels[level] - 3 )  && (lift.getCurrentPosition() >= levels[level]- 3 && lift.getCurrentPosition() <= levels[level] + 3))
        {
            isFinnished  = true;
        }
        else
        {
            isFinnished = false;
        }
        return isFinnished;
    }








    public boolean move(double m) {
        double out = 0;
        if(!opMode.isStopRequested() && opMode.opModeIsActive()){
        int a = levels[level];
        opMode.telemetry.addData("a", a);

        double err = a - lift.getCurrentPosition();

        if (!manual) {
           // pid.setF_togle(level > 0);
            if (err > 0) {
                pid.setF(F);
            } else {
                pid.setF(0);
            }
            out = pid.calculate(levels[level],level);
            // ask s
        } else {

            out = m;
        }

        out = Range.clip(out,-0.8,1);
        pid.setF(F);
        lift.setPower(out);
        }


        opMode.telemetry.addData("ticks", lift.getCurrentPosition());
        opMode.telemetry.addData("l POW", lift.getPower());
        opMode.telemetry.addData("l POW 2", liftTwo.getPower());
        return (Math.abs(out) < 0.06 + pid.getF());
    }

    /**
     * reset the encoders for use
     */
    public void resetEncoders() {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        manual = false;
        level = 0;


    }

    public void setManual() {
        manual = true;
    }


    /**
     * stopping the power and movements of the motors
     */
    //public void stop() {
      //  lift.setPower(0);
     //  liftTwo.setPower(0);


  //  }

    public void reset() {
        resetEncoders();
    }

    public double ticksToMM(double ticks) {
        return ticks / COUNTS_PER_MM;
    }

}