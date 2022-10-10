package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class lift {

    private static final double startingPoint = 1000;
    public ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift = null;
    private DcMotor fourbar = null;

    private int level = 0;
    private int[] levels = {0,100,600,800,1000};
    private int[] fb_levels = {0,100,100,100,100};
    private boolean isBusy = false;
    private Pid Pid;
    private Pid fbPid;
    // private DcMotor lift = null;

    public lift(HardwareMap hw) {
        // private void lifthw(HardwareMap ){

        lift = hw.get(DcMotor.class, "lift");
        lift.setDirection(FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fourbar = hw.get(DcMotor.class,"fourbar");
        fourbar.setDirection(REVERSE);
        fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();

        Pid = new Pid(0.5,0.5 ,0.5);
        Pid.setDirection(false);
        Pid.setMaxIOutput(0.3);
        Pid.setOutputLimits(1);
        Pid.setOutputRampRate(0.5);
        Pid.setSetpointRange(1);

        fbPid = new Pid(0.5,0.5 ,0.5);
        fbPid.setDirection(false);
        fbPid.setMaxIOutput(0.3);
        fbPid.setOutputLimits(1);
        fbPid.setOutputRampRate(0.5);
        fbPid.setSetpointRange(1);
        stop();

    }


    public lift(DcMotor lift) {
        this.lift = lift;
    }


    /**
     * adds 1 the number (of the floor)
     * @param button-said butten to make the change
     */
    public void up(boolean button){
      if (!isBusy && button && level<4) {
        level++;
    }
      isBusy=button;
    }

    /**
     * lowers the number (of the floor) by 1
     * @param button-said butten to make the change
     */
    public void down(boolean button){
       if(!isBusy && button && level>0){
        level--;
    }
       isBusy =button;
    }

    /**
     * transfer the number change of up and down to the number of the arrey
     * then moves the motoor to said position within the arrey
     */
    public void move(){
        int a = levels[level];
        int b = fb_levels[level];
        lift.setPower(Pid.getOutput(lift.getCurrentPosition(),a));
        fourbar.setPower(fbPid.getOutput(fourbar.getCurrentPosition(),b));
    }

    /**
     * reset the encoders for use
     */
    public void resetEncoders(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourbar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * stopping the power and movements of the motors
     */
    public void stop() {
        lift.setPower(0);
        fourbar.setPower(0);
    }
}




