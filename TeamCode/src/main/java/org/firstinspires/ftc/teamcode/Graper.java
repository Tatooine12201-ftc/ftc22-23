package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Graper {
    //INTAKE motors//
    public DcMotorEx grape_right = null;
    public DcMotorEx grape_left = null;

    public Graper(HardwareMap hw) {
        grape_right = hw.get(DcMotorEx.class, "grape_right_motor");
        grape_right.setPower(0);
        grape_left = hw.get(DcMotorEx.class, "grape_left_motor");
        grape_left.setPower(0);

        grape_right.setDirection(DcMotor.Direction.FORWARD);
        grape_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        grape_left.setDirection(DcMotor.Direction.REVERSE);
        grape_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    private static final double INTAKE_SPEED = 1;
    private static final double OUTTAKE_SPEED = -0.5;



    /**
     * this function intakes
     */
    public void intake() {

        grape_right.setPower(INTAKE_SPEED);
        grape_left.setPower(INTAKE_SPEED);
    }

    public void intake(int sec) {
        ElapsedTime spintime = new ElapsedTime();
        spintime.reset();
        while ((spintime.time() < sec)) {
            intake();
        }
        stop();
    }




    public void outtake() {

        grape_right.setPower(OUTTAKE_SPEED);
        grape_left.setPower(OUTTAKE_SPEED);
    }

    public void outtake(int sec) {
        ElapsedTime spintime = new ElapsedTime();
        spintime.reset();
        while ((spintime.time() < sec)) {
            intake();
        }
        stop();
    }

    private void stop() {
        grape_right.setPower(0);
        grape_left.setPower(0);

    }
}


