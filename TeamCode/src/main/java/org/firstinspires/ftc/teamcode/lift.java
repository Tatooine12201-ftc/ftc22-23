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
    private int level = 0;
    private int[] levels = {0,100,600,800,1000};
    private boolean isBusy = false;
    private Pid Pid;
    // private DcMotor lift = null;

    public lift(HardwareMap hw) {
        // private void lifthw(HardwareMap ){

        lift = hw.get(DcMotor.class, "lift");
        lift.setDirection(FORWARD);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
        stop();
        Pid = new Pid(0.5,0.5 ,0.5);
        Pid.setDirection(false);
        Pid.setMaxIOutput(0.3);
        Pid.setOutputLimits(1);
        Pid.setOutputRampRate(0.5);
        Pid.setSetpointRange(1);

    }


    public lift(DcMotor lift) {
        this.lift = lift;
    }

    //public lift (DcMotor lift, double liftingSpeed, double loweringSpeed) {
    //  down_speed = loweringSpeed;
    //  lift_speed = liftingSpeed;
    // }

    /**
     * the start of the robot and what to do in it like
     * seting the current power to zero
     * if the lift isnt in the right position the bring it to it
     * and stop the move ments of the motor
     */
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
       lift.setPower(Pid.getOutput(lift.getCurrentPosition(),a));

    }

    /**
     * reset the encoders for use
     */
    public void resetEncoders(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * stopping the power and movements of the motors
     */
    public void stop() {
        lift.setPower(0);
    }
}


