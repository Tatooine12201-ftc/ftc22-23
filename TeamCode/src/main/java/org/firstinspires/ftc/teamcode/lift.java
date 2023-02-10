package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class lift {
    private final boolean liftIsBusy = false;
    private final int[] levels = {10,100, 840, 1300, 845, 600, 800, 755, 580};

    private final boolean isBusy = false;
    private final boolean isBusy2 = false;
    private final boolean isBusy3 = false;
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor lift = null;
    public DcMotor liftTwo = null;
    int autoHige = 4;
    int autoStack4 = 5;
    int liftStack = 6;
    int autoStack2 = 6;
    int autoStack1 = 7;
    LinearOpMode opMode;

    int prevLevel =0;
    double F =0;

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


    public boolean move(double m) {

        int a = levels[level];

        opMode.telemetry.addData("a", a);
        double out = 0;
        if (!manual) {
            pid.setF_togle(level > 0);
            if (prevLevel > level){
                pid.setF(0);
            }
            else {
                pid.setF(F);
            }
            out = pid.calculate(a - lift.getCurrentPosition());
        } else {

            out = m;
        }

        out = Range.clip(out,-0.8,1);
        lift.setPower(out);
        liftTwo.setPower(out);
        prevLevel =a;



        opMode.telemetry.addData("ticks", lift.getCurrentPosition());
        opMode.telemetry.addData("l POW", lift.getPower());
        opMode.telemetry.addData("l POW 2", liftTwo.getPower());
        opMode.telemetry.update();
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




