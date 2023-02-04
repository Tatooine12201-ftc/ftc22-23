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
    private final int[] levels = {10, 800, 1250, 700, 1250, 990, 755, 580};

    private final boolean isBusy = false;
    private final boolean isBusy2 = false;
    private final boolean isBusy3 = false;
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor lift = null;
    int autoMid = 3;
    int autoStack4 = 4;
    int autoStack3 = 5;
    int autoStack2 = 6;
    int autoStack1 = 7;
    LinearOpMode opMode;

    private int level = 0;
    private boolean manual = false;
    private Pid pid;

    public lift(HardwareMap hw, LinearOpMode opMode) {

        this.opMode = opMode;

        lift = hw.get(DcMotor.class, "lift");
        lift.setDirection(FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        resetEncoders();

        pid = new Pid(0.005, 0.001, 0, 0.15, true);
        //pid = new Pid(0, 0, 0, 0.1,true);
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
            out = pid.calculate(a - lift.getCurrentPosition());
        } else {

            out = m;
        }

        out = Range.clip(out,-0.7,1);
        lift.setPower(out);



        opMode.telemetry.addData("ticks", lift.getCurrentPosition());
        opMode.telemetry.addData("l POW", lift.getPower());
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
    public void stop() {
        lift.setPower(0);


    }

    public void reset() {
        resetEncoders();
    }


}




