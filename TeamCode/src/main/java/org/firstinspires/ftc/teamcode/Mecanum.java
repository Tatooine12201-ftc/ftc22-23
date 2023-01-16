package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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

    private static final double COUNTS_PER_MOTOR_REV = 8192;    // eg: TETRIX Motor Encoder
    //private static final double COUNTS_PER_RADIAN = 6.283185307179586; //
    private static final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private static final double WHEEL_DIAMETER_MM = 35;     // For figuring circumference
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_MM * Math.PI;
    private static final double COUNTS_PER_MM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / WHEEL_CIRCUMFERENCE;
    private static final double F = 21.59;//oders dis from the midle point(x)
    private static final double L = 127.5;//tween encoders
    private static final double X_FORWARD_OFFSET = 143.78;
    private static final double Y_SIDE_OFFSET = 0.15;
    private static final double normalizeRadians = 2 * Math.PI;
    private boolean isBusy = false;
    private boolean isOpen = false;


    double errors[] = new double[3];
    public double startX =0;
    public double startY =0;

<<<<<<< HEAD
     private Pid xPid = new Pid(0.00112, 0.001,0.0455, 0);//0.0451
   // private Pid xPid = new Pid(0, 0,0, 0);//-
    private Pid yPid = new Pid(0.0011, 0.0001, 0.018, 0);
    private Pid rPid = new Pid(0.238, 0, 0.67, 0);//
    //private Pid rPid = new Pid(0, 0, 0, 0);
    //private Pid rPid = new Pid(0.00222, 0, 0, 0);
=======
    private Pid xPid = new Pid(0.05, 0,0, 0);
    private Pid yPid = new Pid(0.0011, 0.0001, 0.018, 0);
    //private Pid rPid = new Pid(1.3, 0.001, 0.07, 0);
    private Pid rPid = new Pid(0.9, 0.001, 0.05, 0);
>>>>>>> parent of 34e27e2 (14.1)

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
    private double robotHading = 0;

    double prvRobotXr = 0;
    double prvRobotXl = 0;
    double prvRobotY = 0;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    double reset =0;


    private double fieldX = 0;
    private double fieldY = 0;

    private double fvStartingPointR = 0;
    private LinearOpMode opMode;
    // private double delXperp = 0;
    public boolean filed = true;

    public Mecanum(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;

        // Parts in hardware map
        flm = hw.get(DcMotorEx.class, "FLM");//y
        blm = hw.get(DcMotorEx.class, "BLM");//xl
        frm = hw.get(DcMotorEx.class, "FRM");//xr
        brm = hw.get(DcMotorEx.class, "BRM");

        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hw.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetEncoders();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);



        flm.setDirection(DcMotorSimple.Direction.FORWARD);
        blm.setDirection(DcMotorSimple.Direction.FORWARD);
        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);

        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();


        // pid config
        //X

<<<<<<< HEAD
        xPid.setMaxIntegral(0.1523);
        xPid.setTolerates(1);
=======
        xPid.setMaxIntegral(0.15);
        xPid.setTolerates(0);
>>>>>>> parent of 34e27e2 (14.1)
        //Y

        yPid.setMaxIntegral(0.22);
        yPid.setTolerates(1);
        //R

        rPid.setMaxIntegral(0.2);
        rPid.setTolerates(Math.toRadians(0.4));
    }

    public double getYe() {
        return (flm.getCurrentPosition());
    }

    public double getXLe() {
        return (-blm.getCurrentPosition());
    }

    public double getXRe() {
        return (-frm.getCurrentPosition());
    }


    /**
     * as the name sugests it resets the encoders
     */
    public void resetEncoders() {
        reset = robotHading;

        robotHading = 0;


        frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    /**
     * as the name sugests gets the x through update
     *
     * @return the fields x
     */

    public double getX() {
        update();
        return fieldX;
    }



    /**
     * as the name sugests gets the y through update
     *
     * @return the fields y
     */

    public double getY() {
        update();
        return  fieldY;
    }


    /**
     * as the name sugests gets the r(rotation)
     *
     * @return the fields r
     */

    public double getFvStartingPointR() {
        return fvStartingPointR;
    }


    /**
     * sets the starting point(x,y,r) the robot starts with
     *
     * @param x
     * @param y
     * @param r
     */

    public void setStartingPoint(double x, double y, double r) {
        robotHading = Math.toRadians(r);
        opMode.telemetry.addData("ssdgfsdkhdf",robotHading);
        opMode.telemetry.update();
        fvStartingPointR = r;
        fieldX = x ;
        fieldY = y ;
        startX = fieldX;
        startY = fieldY;
    }



    /**
     * not as the name sagests update converts the robots x and to the robot y to the fields x and y
     */
    public void update() {




        prvRobotXl = robotXl;
        prvRobotXr = robotXr;
        prvRobotY = robotY;

        robotXr = ticksToMM(getXRe());
        robotXl = ticksToMM(getXLe());
        robotY = ticksToMM(getYe());
        double heading = heading();


        double deltaLeft = robotXl - prvRobotXl;
        double deltaRight = robotXr - prvRobotXr;
        double daltaY  = robotY - prvRobotY;

        double phi = (deltaLeft - deltaRight) / L;
        double deltaX  = (deltaLeft + deltaRight) / 2;
        double daltay = daltaY - F * phi;



        fieldX += deltaX * Math.cos(heading) - daltay * Math.sin(heading);
        fieldY += deltaX * Math.sin(heading) + daltay * Math.cos(heading);
    }


    /**
     * basic ticks to mm convertor
     *
     * @param ticks
     * @return
     */
    private double ticksToMM(double ticks) {
        return ticks / COUNTS_PER_MM;
    }

    public void worldtorobot(double x, double y, double r) {
        double deltaX = x - getX();
        double deltaY = y - getY();


       // double  xToMove = deltaX * Math.cos(botHeading) - deltaY * Math.sin(botHeading);
       // double  yToMove = deltaX * Math.sin(botHeading) + deltaY * Math.cos(botHeading);

       double head = heading();

        double xToMove = deltaX * Math.cos(head) + deltaY * Math.sin(head);
        double yToMove  = deltaX * Math.sin(head) - deltaY * Math.cos(head);
        errors[1] = yToMove;
        errors[0] = xToMove;

        errors[2] = Math.toRadians(r) - head;
        }




    public void driveTo(double x , double y , double r){
        double xPow = 1;
        double yPow = 1;
        double rPow = 1;
        while ((xPow!=0 || yPow!=0 || rPow!=0) && (opMode.opModeIsActive() && !opMode.isStopRequested())){
            worldtorobot(x,y,r);
            xPow = xPid.calculate(errors[0]);
            yPow = yPid.calculate(errors[1]);
            rPow = rPid.calculate(errors[2]);
            filed = false;
            drive(yPow,xPow,rPow);
          //  drive(0 ,0,rPow);


            opMode.telemetry.addData("x",errors[0] );
            opMode.telemetry.addData("y",errors[1] );
            opMode.telemetry.addData("r",Math.toDegrees(errors[2] ));

            opMode.telemetry.addData("ypow",yPow);
            opMode.telemetry.addData("xpow",xPow);
            opMode.telemetry.addData("rpow",rPow);

            opMode.telemetry.addData("FX",getX());
            opMode.telemetry.addData("FY",getY());
            opMode.telemetry.addData("head", headingToDegrees());



            opMode.telemetry.update();
            //opMode.sleep(1000);


        }
        filed = true;
    }

    public void drive(double x, double y, double r) {
        // Read inverse IMU heading, as the IMU heading is CW positive
        update();
        double botHeading = heading();
        double rotX = 0;
        double rotY = 0;

        if (filed) {


            rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        } else {
            rotX = x;
            rotY = y;
        }


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(r), 1);
        double frontLeftPower = (rotY + rotX + r) / denominator;
        double backLeftPower = (rotY - rotX + r) / denominator;
        double frontRightPower = (rotY - rotX - r) / denominator;
        double backRightPower = (rotY + rotX - r) / denominator;




        flm.setPower(frontLeftPower);
        blm.setPower(backLeftPower);
        frm.setPower(frontRightPower);
        brm.setPower(backRightPower);
    }


        public void changePosition(boolean button){
            if (!isBusy && !filed && button){
                filed = true;
            }
            else if (!isBusy && filed && button ){
                filed=false;
            }
            isBusy = button;
        }









    public double heading() {
        robotHading =  (-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle) + fvStartingPointR - reset;

        return normalizeRadians(robotHading);
        //  return //normalizeRadians(robotHading);-imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }



    public double headingToDegrees() {
        return Math.toDegrees(heading());
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

    private static double normalizeRadians(double radians){
        if (radians > Math.PI) {
            return radians - normalizeRadians;
        } else if (radians < -Math.PI) {
            return radians + normalizeRadians;
        } else return radians;
    }
    public String toString() {
        String out = String.format("flm: %f blm: %f frm: %f brm: %f \nEncoders  flm : %f blm : %f  frm :%f brm : %f ", flm.getPower(), blm.getPower(), frm.getPower(), brm.getPower(),flm.getCurrentPosition(),blm.getCurrentPosition(),frm.getCurrentPosition(),brm.getCurrentPosition());
        return out;
    }
}














