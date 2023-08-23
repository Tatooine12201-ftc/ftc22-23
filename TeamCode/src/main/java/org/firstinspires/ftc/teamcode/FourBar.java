package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar {

    private DcMotor FourbarMotor = null;

    public FourBar(DcMotor fourbarMotor){

        FourbarMotor = fourbarMotor;
        FourbarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public DcMotor getFourbarMotor() {
        return FourbarMotor;
    }

    public  DcMotor setFourbarMotor(DcMotor fourbarMotor){
        FourbarMotor = fourbarMotor
    }
}

