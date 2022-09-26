package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class Pliers {

public Servo Pliers = null;
private boolean isBusy = false;
private boolean isOpen = false;
    public Pliers(HardwareMap hw) {
        // private void lifthw(HardwareMap ){
        Pliers = hw.get(Servo.class, "Pilier");
        Pliers.setPosition(0);
       // Pilier.;
        //stop();
    }
    public void Open (){
        Pliers.setPosition(1);
        isOpen = true;
    }
    public void close (){
        Pliers.setPosition(0);
        isOpen = false;
    }
    public void changePosition(boolean button){
     if (!isBusy && !isOpen && button){
         Open();
     }
     else if (!isBusy && isOpen && button ){
         close();
     }
    isBusy = button;
    }
}
