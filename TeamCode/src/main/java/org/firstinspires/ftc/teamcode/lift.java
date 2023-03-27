package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class lift  {
    //Thread lift_thread = new Thread();
    private final boolean liftIsBusy = false;

    private static double GEAR_RATIO = 20.0/1.0;
    public static double TICKS_PER_REV = 28;
    public static double PULY_PERIMITAR = 78.5;
 // private static final double COUNTS_PER_MM = 3.75;
    private static final double COUNTS_PER_MM =GEAR_RATIO* TICKS_PER_REV /PULY_PERIMITAR;
    private final int[] levels = {
            20,//0
            100,//1
            840,//2
            1300,//3

            1100,//4

            520,//5
            400,//6 //800
            300,//7
            200,//8

            450 ,//9

    };

    private final boolean isBusy = false;
    private final boolean isBusy2 = false;
    private final boolean isBusy3 = false;
    public ElapsedTime runtime = new ElapsedTime();
    public boolean done;
    public DcMotor lift = null;
    public DcMotor liftTwo = null;
    int autoHige = 4;

    int autoStack4 = 5;
    int autoStack3 = 6;
    int autoStack2 = 7;
    int autoStack1 = 8;
    int aboveWheel = 9;
    LinearOpMode opMode;

    int prevLevel = 0;
    double F = 0;

    private int level = 0;
    private boolean manual = false;
    private Pid pid;


    public lift(HardwareMap hw, LinearOpMode opMode) {


        this.opMode = opMode;

        lift = hw.get(DcMotor.class, "lift");
        lift.setDirection(FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





        resetEncoders();

        pid = new Pid(0.005,0.0001,0,0);

        F = pid.getF();

        pid.setIntegrationBounds(-0.27,0.27);
        pid.setTolerance(20);


        stopm();

    }
    public double ticksToMM(double ticks) {
        return ticks / COUNTS_PER_MM;
    }
    public double MMToTicks(double MM){return MM * COUNTS_PER_MM;}

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












    public boolean move(double m) {

        double target = levels[level];
        double out;

        if (!manual) {
           out = pid.calculate(getEncoder(),target);

        } else {

            out = m;
        }


        lift.setPower(out);

//48
        //23
        done = pid.atSetPoint();

        return (pid.atSetPoint());
        // chaengh the out
    }

    public boolean isatpos (){
        return (pid.atSetPoint());
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



}