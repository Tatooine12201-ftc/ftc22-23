package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class servo_new {

    public servo_new (HardwareMap hw ) {
        //get our analog input from the hardwareMap
        AnalogInput analogInput = hw.get(AnalogInput.class, "myanaloginput");
        ServoImplEx servo = hw.get(ServoImplEx.class, "myservo");

        // get the voltage of our analog line
// divide by 3.3 (the max voltage) to get a value between 0 and 1
// multiply by 360 to convert it to 0 to 360 degrees
        double position = analogInput.getVoltage() / 3.3 * 360;

// axon servos have a PWM range of 500us-2500us
// but we keep it slightly lower to ensure no wraparounds occur
        servo.setPwmRange(new PwmControl.PwmRange(505, 2495));

// set position
        servo.setPosition(1);


    }



}
