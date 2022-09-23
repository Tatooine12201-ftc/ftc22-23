package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class lift {
    private static final double lift_speed = 0.7;
    private static final double down_speed = -0.5;
    private static final double startingPoint = 1000;
    public ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    // private DcMotor lift = null;

    public lift(HardwareMap hw) {
        // private void lifthw(HardwareMap ){

        lift = hw.get(DcMotor.class, "lift");
        lift.setDirection(FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stop();

    }


    public lift(DcMotor lift) {
        this.lift = lift;
    }

    //public lift (DcMotor lift, double liftingSpeed, double loweringSpeed) {
    //  down_speed = loweringSpeed;
    //  lift_speed = liftingSpeed;
    // }
    public void init() {
        lift.setPower(0);
        lift.getCurrentPosition();
        if (lift.getCurrentPosition() > startingPoint) {
            lift.setPower(down_speed);
        } else if (lift.getCurrentPosition() < startingPoint) {
            lift.setPower(lift_speed);
        } else {
            stop();

        }
    }

    public void goToThird() {
        if (lift.getCurrentPosition() < 5000) {
            lift.setPower(lift_speed);
        } else {
            stop();
        }
    }

    public void goToSecond() {
        if (lift.getCurrentPosition() > 2500) {
            lift.setPower(down_speed);
        } else if (lift.getCurrentPosition() < 2500) {
            lift.setPower(lift_speed);
        } else {
            stop();
        }
    }

    public void goToFirst() {
        if (lift.getCurrentPosition() > 0) {
            lift.setPower(down_speed);
        } else {
            stop();

        }
    }

    public void goToStartingPoint() {
        if (lift.getCurrentPosition() > startingPoint) {
            lift.setPower(down_speed);
        } else if (lift.getCurrentPosition() < startingPoint) {
            lift.setPower(lift_speed);
        } else {
            stop();
        }
    }

    public void stop() {
        lift.setPower(0);
    }
}

