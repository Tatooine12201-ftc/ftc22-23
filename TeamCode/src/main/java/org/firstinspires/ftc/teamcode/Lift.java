package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Lift {
    private DcMotor LiftMotor = null;

    public Lift(DcMotor liftMotor) {
        LiftMotor = liftMotor;
    }

    public DcMotor getLiftMotor() {
        return LiftMotor;
    }

    public void setLiftMotor(DcMotor liftMotor) {
        LiftMotor = liftMotor;
    }

    public void lift(double power){
        LiftMotor.setPower(power);
    }
}
