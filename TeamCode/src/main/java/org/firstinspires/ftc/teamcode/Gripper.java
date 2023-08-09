package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    private Servo grepperServo;
    private boolean isOpen = true;

    public Gripper(Servo grepperServo) {
        this.grepperServo = grepperServo;
    }

    public Servo getGrepperServo() {
        return grepperServo;
    }

    public void setGrepperServo(Servo grepperServo) {
        this.grepperServo = grepperServo;
    }

    public void open(){
        grepperServo.setPosition(1);
        isOpen = true;
    }

    public void close(){
        grepperServo.setPosition(0);
        isOpen = false;
    }

    public void change(){
        if(isOpen){
            close();
        }
        else {
            open();
        }
    }
}
