package org.firstinspires.ftc.teamcode;


import android.os.Build;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.time.Duration;
import java.time.Instant;
import java.util.Timer;

/**
 * the PIDController class will be used for  all the different pid calculations
 * for example AutoDriving ,autoAdjusting , controlling the shooter speed
 */
//P is main power, I looks at the sum of error and gives final push, D is how much the error is changing
public class Pid {
    boolean F_togle;

    //TODO look at using System instead of ElapsedTime
    private long start = System.nanoTime();
    boolean IzoneDisabeled = false;
    double tolerates = 0;
    double integral = 0;
    double derivative = 0;
    private double maxI = 0;
    private double Izone = 0;
    private long previousTime;
    private double kp;
    private double ki;
    private double kd;
    private double f;

    private double p =0;
    private double i = 0;
    private double d = 0;



    public Pid(double kp, double ki, double kd, double f,double Izone ,boolean f_togle) {
        this.F_togle = f_togle;
        start = System.nanoTime();

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.f = f;
    }

    public Pid(double kp, double ki, double kd, double f, double Izone) {
        start = System.nanoTime();
        this.Izone = Izone;
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.f = f;
    }



    public double getTolerates() {
        return tolerates;
    }

    public void setTolerates(double tolerates) {
        this.tolerates = tolerates;
    }

    public double getMaxIntegral() {
        return maxI;
    }

    public void setMaxIntegral(double maxI) {
        this.maxI = maxI;
    }

    public boolean getFt() {
        return F_togle;
    }

    public void setF_togle(boolean f_togle) {
        this.F_togle = f_togle;
    }

    public double getP() {
        return p;
    }

    public void setP(double p) {
        this.p = p;
    }

    public double getI() {
        return i;
    }

    public void setI(double i) {
        this.i = i;
    }

    public double getD() {
        return d;
    }

    public void setD(double d) {
        this.d = d;
    }

    public double getF() {
        return f;
    }

    public void setF(double f) {
        this.f = f;
    }

    public void DisabeleIzone() {
        IzoneDisabeled = true;
    }

    public double calculate(double error, long currentTime) {
        p =0;
        i =0;
        d=0;
        if (Math.abs(error) <= tolerates) {
            if (F_togle) {
                return f;
            }
            return 0;
        }


         p = kp * error;

        if (Math.abs(error) < Izone || IzoneDisabeled) {

            i = Range.clip(integral + ki * (error * (currentTime - previousTime)), -maxI, maxI);

            integral = i;
        }




            d = kd * ((derivative- error) / (previousTime - currentTime));

            derivative = error;
            previousTime = currentTime;



        return f + p + i + d;
    }
}