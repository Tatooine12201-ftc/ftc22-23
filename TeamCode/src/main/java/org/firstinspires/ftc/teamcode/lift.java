package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class lift {
    private final boolean liftIsBusy = false;
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
    ElapsedTime time =new ElapsedTime();
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

        liftTwo = hw.get(DcMotor.class, "liftTwo");
        liftTwo.setDirection(REVERSE);
        liftTwo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        resetEncoders();

        pid = new Pid(0.005, 0.0001, 0, 0.19, true);
        F = pid.getF();

        pid.setMaxIntegral(0.27);
        pid.setTolerates(10);


        stop();

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


    public boolean moveTo(int level) {
        //the default timeout is 5 seconds
        return moveTo(level, 3000);
    }
    public boolean moveTo(int level, double timeOut) {
        boolean done = false;
        double startTime = time.milliseconds();
        do {

            //check if the robot has tried for more than timeOut milliseconds
            if (time.milliseconds() - startTime > timeOut ) {
                lift.setPower(0);
                liftTwo.setPower(0);
                //return false
                opMode.telemetry.clearAll();
                opMode.telemetry.addData("timeOut", "timeOut");

                return false;
            }
            setLevel(level);
            pid.setF(F);
            done = move(0);
        }while (!done);
        pid.setF(F);
        move(0,true);
        return true;
    }
    public boolean move(double m){
        return move(m,false);
    }





    public boolean move(double m,boolean f) {
        double out = 0;
        if(!opMode.isStopRequested() && opMode.opModeIsActive()){
        int a = levels[level];
        opMode.telemetry.addData("a", a);

        double err = a - lift.getCurrentPosition();

        if (!manual) {
            pid.setF_togle(level > 0);
            if (err > 0) {
                pid.setF(F);
            } else {
                pid.setF(0);
            }
            out = pid.calculate(err);
        } else {

            out = m;
        }

        out = Range.clip(out,-0.8,1);
        lift.setPower(out);
        liftTwo.setPower(out);}
        else {
            stop();
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
        liftTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftTwo.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        manual = false;
        level = 0;


    }

    public void setManual() {
        manual = true;
    }


    /**
     * stopping the power and movements of the motors
     */
    public void stop() {
        lift.setPower(0);
        liftTwo.setPower(0);


    }

    public void reset() {
        resetEncoders();
    }


}