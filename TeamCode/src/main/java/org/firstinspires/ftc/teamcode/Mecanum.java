package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.*;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;



public class Mecanum {

    private static final double COUNTS_PER_MOTOR_REV = 8192 ;    // eg: TETRIX Motor Encoder
    //private static final double COUNTS_PER_RADIAN = 6.283185307179586; //
    private static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_MM = 35;     // For figuring circumference
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_MM * Math.PI;
    private static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;
    private static final double F = 21.59;//oders dis from the midle point(x)
    private static final double L =127.5;//tween encoders
    private static final double X_FORWARD_OFFSET =143.78;
    private static final double Y_SIDE_OFFSET = 0.15;

    private Pid yPid;
    private Pid xPid;
    private Pid rPid;

    //private static final double COUNTS_PER_DE = (COUNTS_PER_RADIAN * 180/Math.PI) ;
    //DRIVE motors//
    private BNO055IMU imu = null;
    private DcMotorEx flm = null;
    private DcMotorEx blm = null;
    private DcMotorEx frm = null;
    private DcMotorEx brm = null;

    private double robotXr = 0;
    private double robotXl = 0;
    private double robotY = 0;
    private double robotHading =0;

    double prvRobotXr = robotXr;
    double prvRobotXl = robotXl;
    double prvRobotY = robotY;


    private double fieldX = 0;
    private double fieldY = 0;

    private double fvStartingPointR = 0;

   // private double delXperp = 0;

    public Mecanum(HardwareMap hw) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        // Parts in hardware map
        imu = hw.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        flm = hw.get(DcMotorEx.class, "FLM");//y
        blm = hw.get(DcMotorEx.class, "BLM");//xl
        frm = hw.get(DcMotorEx.class, "FRM");//xr
        brm = hw.get(DcMotorEx.class, "BRM");

        flm.setDirection(DcMotorSimple.Direction.FORWARD);
        blm.setDirection(DcMotorSimple.Direction.FORWARD);
        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();

        // pid config
        //X
        xPid = new Pid(0.5,0.5 ,0.5);
        xPid.setDirection(false);
        xPid.setMaxIOutput(0.3);
        xPid.setOutputLimits(1);
        xPid.setOutputRampRate(0.5);
        xPid.setSetpointRange(1);
        //Y
        yPid = new Pid(0.5,0.5 ,0.5);
        yPid.setDirection(false);
        yPid.setMaxIOutput(0.1);
        yPid.setOutputLimits(1);
        yPid.setOutputRampRate(0.5);
        yPid.setSetpointRange(1);
        //R
        rPid = new Pid(0.5,0.5 ,0.5);
        rPid.setDirection(false);
        rPid.setMaxIOutput(0.3);
        rPid.setOutputLimits(1);
        rPid.setOutputRampRate(0.5);
        rPid.setSetpointRange(1);
    }

    public double getYe(){
        return (flm.getCurrentPosition());
    }

    public double getXLe(){
        return blm.getCurrentPosition();
    }

    public double getXRe(){
        return frm.getCurrentPosition();
    }


    /**
     * as the name sugests it resets the encoders
     */
    public void resetEncoders(){
        frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fvStartingPointR = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    }

    /**
     * as the name sugests gets the x through update
     * @return the fields x
     */

    public double getX() {
        update();
        return fieldX + X_FORWARD_OFFSET;
    }

    /**
     * as the name sugests it sets the starting point and converts it to a x the robot can understand
     * @param fStartingPointX
     */

    public void setStartingPointX(double fStartingPointX) {

        this.fieldX = fStartingPointX;
        double offset = Math.sin(Math.toRadians(fvStartingPointR));
        robotXr = (fieldX * Math.cos(Math.toRadians(fvStartingPointR))) - (fieldY * Math.sin(Math.toRadians(fvStartingPointR) + offset));
        robotXl = (fieldX * Math.cos(Math.toRadians(fvStartingPointR))) - (fieldY * Math.sin(Math.toRadians(fvStartingPointR) - offset));
    }
    /**
     * as the name sugests gets the y through update
     * @return the fields y
     */

    public double getY() {
        update();
        return fieldY + Y_SIDE_OFFSET;
    }
    /**
     * as the name sugests it sets the starting point and converts it to a y the robot can understand
     * @param fStartingPointY
     *
     *
     *
     *
     */
    public void setStartingPointY(double fStartingPointY) {
        this.fieldY = fStartingPointY;
        robotY = (fieldX * Math.sin(Math.toRadians(fvStartingPointR))) + (fieldY * Math.cos(Math.toRadians(fvStartingPointR) ));
    }
    /**
     * as the name sugests gets the r(rotation)
     * @return the fields r
     */

    public double getFvStartingPointR() {
        return fvStartingPointR;
    }

    /**
     * as the name sugests sets the starting r(rotation) the robot works with
     * @param fvStartingPointR
     */

    public void setFvStartingPointR(double fvStartingPointR) {
        this.fvStartingPointR = fvStartingPointR;
    }

    /**
     * sets the starting point(x,y,r) the robot starts with
     * @param x
     * @param y
     * @param r
     */

    public void setStartingPoint(double x, double y, double r) {
        setFvStartingPointR(r);
        setStartingPointX(x);
        setStartingPointY(y);
    }
    public void newsetStartingPoint(double x, double r) {
        setFvStartingPointR(r);
        setStartingPointX(x);

    }

    /**
     * not as the name sagests update converts the robots x and to the robot y to the fields x and y
     */
    public void update() {


        robotXr = ticksToMM(getXRe());
        robotXl = ticksToMM(getXLe());
        robotY = ticksToMM(getYe());

        double deltaLeft = robotXl - prvRobotXl;
        double deltaRight = robotXr - prvRobotXr;
        double daltaY = robotY - prvRobotY;

        double phi = (deltaLeft - deltaRight) / L;
        double robotXC = (deltaLeft + deltaRight) / 2;
        double roboty =  daltaY - F * phi;

        double deltaFildeX = robotXC * Math.cos(headingInRed()) - roboty * Math.sin(headingInRed());
        double deltaFildeY = robotXC * Math.sin(headingInRed()) + roboty * Math.cos(headingInRed());
        fieldX += deltaFildeX;
        fieldY += deltaFildeY;
        robotHading += phi;


        prvRobotXl = ticksToMM(getXLe());
        prvRobotXr = ticksToMM(getXRe());
        prvRobotY = ticksToMM(getYe());
    }




    /**
     * basic ticks to mm convertor
     * @param ticks
     * @return
     */
    private double ticksToMM(double ticks) {
        return ticks / COUNTS_PER_MM;
    }

    public double[]  worldtorobot (double x ,double y, double r){
        double offset = Math.sin(Math.toRadians(r));

        double xn = (x * Math.cos(Math.toRadians(r))) - (y * Math.sin(Math.toRadians(r)));
        double yn = (x * Math.sin(Math.toRadians(r))) + (y * Math.cos(Math.toRadians(r)));
        double[] pos = {xn, yn};
        return pos;
    }

    public void driveTo(double x , double y , double r){
        double xPow = 1;
        double yPow = 1;
        double rPow = 1;

        while ((xPow != 0) && (yPow != 0) && (rPow != 0)){
            double[] pos = worldtorobot(x,y,heading());
           xPow = xPid.getOutput(getX(), Range.clip(pos[0],0, 3657.6));
           yPow = yPid.getOutput(getY(), Range.clip(pos[1], 0 , 3657.6));
           rPow = rPid.getOutput(heading(),normalizeDegrees(r));
           drive(xPow,yPow,rPow,false);
        }
    }

    public void drive(double x, double y, double r, boolean squaredInputs) {
        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = headingInRed();

        if ( Math.abs(x)<0.02){
            x=0;
        }

        if ( Math.abs(y)<0.02){
            y=0;
        }

        if ( Math.abs(r)<0.02){
            r=0;
        }
        double newR = r;
        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max((Math.abs(y) + Math.abs(x) + Math.abs(r)), 1);
        double frontLeftPower = (rotY + rotX + newR) / denominator;
        double backLeftPower =(rotY - rotX + newR) / denominator;
        double frontRightPower = (rotY - rotX - newR) / denominator;
        double backRightPower = (rotY + rotX - newR) / denominator;


        flm.setPower(frontLeftPower);
        blm.setPower(backLeftPower);
        frm.setPower(frontRightPower);
        brm.setPower(backRightPower);
        update();

    }


    public double heading() {
        return (robotHading);///-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public double headingInRed() {
        return Math.toRadians(heading());
    }

    public void stop() {
        flm.setPower(0);
        blm.setPower(0);
        frm.setPower(0);
        brm.setPower(0);
    }

    public void setZeroBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        flm.setZeroPowerBehavior(zeroPowerBehavior);
        blm.setZeroPowerBehavior(zeroPowerBehavior);
        frm.setZeroPowerBehavior(zeroPowerBehavior);
        brm.setZeroPowerBehavior(zeroPowerBehavior);
    }
    private static double normalizeDegrees(double degrees){
        double temp = (degrees + 180.0) % 360.0;
        return temp - 180.0;
    }
    public String toString() {
        String out = String.format("flm: %f blm: %f frm: %f brm: %f", flm.getPower(), blm.getPower(), frm.getPower(), brm.getPower());
        return out;
    }
}














