package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class lift {
    int autoMid = 3;
    int autoStack4 = 4;
    int autoStack3 = 5;
    int autoStack2 = 6;
    int autoStack1 = 7;

    private static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    //private static final double COUNTS_PER_RADIAN = 6.283185307179586; //
    private static final double DRIVE_GEAR_REDUCTION = 1.0 / 40.0 * 15.0 / 125.0;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_deg = (COUNTS_PER_MOTOR_REV / 360 * DRIVE_GEAR_REDUCTION);

    public ElapsedTime runtime = new ElapsedTime();
    LinearOpMode opMode;
    private DcMotor lift = null;
    private final boolean liftIsBusy = false;
    private int level = 0;
    private final int[] levels = {0, 2150, 2990,700,1250,990,755,580};
    private final boolean isBusy = false;
    private final boolean isBusy2 = false;
    private final boolean isBusy3 = false;
    private boolean manual = false;
    private Pid pid;

    public lift(HardwareMap hw, LinearOpMode opMode) {

        this.opMode = opMode;

        lift = hw.get(DcMotor.class, "lift");
        lift.setDirection(FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        resetEncoders();

        pid = new Pid(0.0014, 0.00001, 0, 0.11,0);
        pid.setMaxIntegral(0.15);
        pid.setTolerates(10);
        pid.DisabeleIzone();

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


    /**
     * adds 1 the number (of the floor)
     * @param button-said butten to make the change
     */


    /**
     * lowers the number (of the floor) by 1
     * @param button-said butten to make the change
     */


    /**
     * transfer the number change of up and down to the number of the arrey
     * then moves the motoor to said position within the arrey
     */
    public boolean move(double m) {

        int a = levels[level];
        opMode.telemetry.addData("a",a);
        double out = 0;
        if (!manual) {
            if(level > 0)
            {
                pid.setF_togle(true);
            }
            else {
                pid.setF_togle(false);
            }
            out = pid.calculate(a - lift.getCurrentPosition(),System.nanoTime());
        } else {

            out = m;
        }


        lift.setPower(out);


        opMode.telemetry.addData("ticks", lift.getCurrentPosition());
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

    public void setLift(int sec) {
        ElapsedTime spintime = new ElapsedTime();
        spintime.reset();
        while (spintime.time() < sec) {
            setLevel(4);
        }
        stop();
    }
}




