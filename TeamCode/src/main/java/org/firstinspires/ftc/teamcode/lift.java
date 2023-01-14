package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


public class lift {

    private static final double COUNTS_PER_MOTOR_REV = 1440 ;    // eg: TETRIX Motor Encoder
    //private static final double COUNTS_PER_RADIAN = 6.283185307179586; //
    private static final double DRIVE_GEAR_REDUCTION = 1.0/40.0 * 15.0/125.0;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_deg = (COUNTS_PER_MOTOR_REV /360* DRIVE_GEAR_REDUCTION);
   // private static final double startingPoint = 1000;
    public ElapsedTime runtime = new ElapsedTime();
    private DcMotor lift =null;
   // private DcMotor fourbar = null;


    //private boolean fbIsBusy = false;
    private boolean liftIsBusy = false;

    private int level = 0;
    private int[] levels = {0,1500,2500,3000,3320 };
   // private int[] fb_levels = {0, 0,120 * COUNTS_PER_deg,120 * COUNTS_PER_deg,120 *COUNTS_PER_deg};
    private boolean isBusy = false;
    private boolean isBusy2 = false;
    private boolean isBusy3 = false;
    private Pid pid;
   // private Pid fbPid;
    //private DcMotor lift = null;
    LinearOpMode opMode;

    public lift(HardwareMap hw, LinearOpMode opMode) {
        // private void lifthw(HardwareMap ){
        this.opMode = opMode;

        lift = hw.get(DcMotor.class, "lift");
        lift.setDirection(REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



       // fourbar = hw.get(DcMotor.class,"fourbar");
      //  fourbar.setDirection(FORWARD);

        //fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();

        pid = new Pid(0.0016,0.00015,0.2572,0);
        pid.setMaxIntegral(0.01685);
        pid.setTolerates(75);


        //fbPid = new Pid(0,0,0,0);
       // fbPid.setMaxIntegral(0.15);
        //fbPid.setTolerates(20);
        stop();



    }


    public lift(DcMotor lift) {
        this.lift = lift;
    }

    public void setLevel(int level){
        if (level >= 0 && level <= 3){
            this.level = level;
        }
    }

    public int getLevel(){
        return this.level;
    }
    public int getEncoder(){
        return lift.getCurrentPosition();
    }


    /**
     * adds 1 the number (of the floor)
     * @param button-said butten to make the change
     */

 //   public void up(boolean button)
 //   {

 //       if(!liftIsBusy && button && level<4){
 //           level++;
 //           liftIsBusy = true;
 //       }
 //       liftIsBusy = button;

 //   opMode.telemetry.addData("a", isBusy);
       // opMode.telemetry.addData("b", button);
 //       opMode.telemetry.update();
 //   }

    /**
     * lowers the number (of the floor) by 1
     * @param button-said butten to make the change
     */
 //   public void down(boolean button){
 //      if(!isBusy && button && level>0){
 //       level--;
 //       isBusy =true;
 //   }
 //      isBusy = button;
 //   }





    /**
     * transfer the number change of up and down to the number of the arrey
     * then moves the motoor to said position within the arrey
     */
    public boolean move(){
        int a = levels[level];
       // int b = fb_levels[level];

        double out =pid.calculate(a - lift.getCurrentPosition());
      // double fbout =fbPid.calculate(b - fourbar.getCurrentPosition());
        lift.setPower(out);
       // fourbar.setPower(fbout);
        //fourbarRight.setPower(fbout);
        opMode.telemetry.addData("lift", lift.getCurrentPosition());
        opMode.telemetry.addData("level",level);
        opMode.telemetry.addData("out",out);
      //  opMode.telemetry.addData("fbout",fbout);
        opMode.telemetry.addData("max integral",pid.getMaxIntegral());
      //opMode.telemetry.update();
        return (out == 0 );
    }

    /**
     * reset the encoders for use
     */
    public void resetEncoders(){
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //fourbarRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //fourbarRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // fourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // fourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

  //  public void changeDiraction (boolean botun ){
     // for (int i = 0; i < fb_levels.length; i++){
      //    fb_levels[i]= fb_levels[i] * -1;
     // }
     // fbIsBusy = botun;
    //}

    /**
     * stopping the power and movements of the motors
     */
    public void stop() {
        lift.setPower(0);
       // fourbar.setPower(0);

    }

    public void reset() {

    }
    public void setLift (int sec){
        ElapsedTime spintime = new ElapsedTime();
        spintime.reset();
        while (spintime.time() < sec){
            setLevel(4);
        }
        stop();
    }
}




