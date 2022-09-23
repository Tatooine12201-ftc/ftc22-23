package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class intake {
    //INTAKE motors//
    public DcMotorEx intake = null;

    public intake(HardwareMap hw) {
        intake = hw.get(DcMotorEx.class, "intake_motor");
        intake.setPower(0);
        
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    private static final double INTAKE_SPEED = 1;
    private static final double OUTTAKE_SPEED = 0.5;


    /**
     * this function intakes
     */
    public void intake() {

        intake.setPower(INTAKE_SPEED);
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

        intake.setPower(OUTTAKE_SPEED);
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
        intake.setPower(0);

    }
}


