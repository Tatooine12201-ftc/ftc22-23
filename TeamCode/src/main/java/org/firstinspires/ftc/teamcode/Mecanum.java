package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sin;


import android.os.Build;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;


import java.util.concurrent.CompletableFuture;


public class Mecanum {
    //flm -> front left motor -> Y encoder
    //blm -> back left motor -> X left encoder
    //frm -> front right motor -> X right encoder



    private static final double TPI = Math.PI * 2;
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_DIAMETER = 35;
    private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static double GEAR_RATIO = 1;
    private static final double COUNTS_PER_MM = (TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRCUMFERENCE;
    public static double LATERAL_DISTANCE = 127.5;
    public static double FORWARD_OFFSET = 20.97;
    public static double X_OFFSET = -143.85;
     public static double Y_OFFSET = 0;


    private final Pid xPid = new Pid(0.00206, 0.0005, 0.32, 0);

     private final Pid yPid = new Pid(0.0026, 0.0001, 0, 0);

    private final Pid rPid = new Pid(0.75, 0.00068, 0.001, 0);

    private final double fvStartingPointR = 0;

    private final LinearOpMode opMode;
    public double startX = 0;

    public double startY = 0;
    public double Time = 0;
    public double startR = 0;
    public boolean field = true;
    public boolean wasChanged = false;
    double[] errors = new double[2];
    double prevRightEncoderPos = 0;
    double prevLeftEncoderPos = 0;
    double prevCenterEncoderPos = 0;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    double reset = 0;
    boolean wasReset = false;
    //DRIVE motors//
    private BNO055IMU imu = null;
    private DcMotorEx flm = null;
    private DcMotorEx blm = null;
    private DcMotorEx frm = null;
    private DcMotorEx brm = null;
    private double robotHading_CWP = 0;
    private double robotHading_CCWP = 0;
    private double fieldX = 0;
    private double fieldY = 0;
    ElapsedTime time =new ElapsedTime();
    boolean dotimeout = true;

    double wantedAngle =0;

    public Mecanum(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;

        // Parts in hardware map


        flm = hw.get(DcMotorEx.class, "FLM");//y
        blm = hw.get(DcMotorEx.class, "BLM");//xl
        frm = hw.get(DcMotorEx.class, "FRM");//xr
        brm = hw.get(DcMotorEx.class, "BRM");

        //imu
        imu = hw.get(BNO055IMU.class, "imu");
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);


        reset();


        flm.setDirection(DcMotorSimple.Direction.FORWARD);
        blm.setDirection(DcMotorSimple.Direction.FORWARD);
        frm.setDirection(DcMotorSimple.Direction.REVERSE);
        brm.setDirection(DcMotorSimple.Direction.REVERSE);

        brm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // pid config
        //X


        xPid.setMaxIntegral(0.16);
        xPid.setTolerates(20);


        //Y

        yPid.setMaxIntegral(0.2);
        yPid.setTolerates(20);

        //R

        rPid.setMaxIntegral(0.2);
        rPid.setTolerates(Math.toRadians(2));

    }

    public void setStartPos(double x, double y, double r) {
        //set the start position
        startX = x;
        startY = y;
        startR = Math.toRadians(r);
        //set the field position
        fieldX = x;
        fieldY = y;
    }

    /**
     * convert ticks to mm
     *
     * @param ticks ticks
     * @return mm
     */
    public double ticksToMM(double ticks) {
        return ticks / COUNTS_PER_MM;
    }

    //getter and setter
    // encoder

    /**
     * get the middle encoder
     *
     * @return middle encoder
     */
    public double getYEncoder() {
        return flm.getCurrentPosition();
    }

    /**
     * get the right encoder
     *
     * @return right encoder
     */
    public double getXrEncoder() {
        return -frm.getCurrentPosition();
    }


    /**
     * get the left encoder
     *
     * @return left encoder
     */
    public double getXlEncoder() {
        return -blm.getCurrentPosition();
    }

    /**
     * get the field x
     *
     * @return field x
     */
    public double getFieldX() {
        return fieldX;
    }

    /**
     * get the field y
     *
     * @return field y
     */
    public double getFieldY() {
        return fieldY;
    }

    /**
     * reset the encoders and imu
     */
    public void reset() {
        //reset the encoders that are used for Xr, Xl and Y
        flm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //reset the imu (cw is positive)
        reset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;

    }

    public void setAngle(double angle, boolean x) {
        //set the angle of the robot
        if (x && !wasReset) {
            reset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - angle;
            wasReset = true;
        } else {
            wasReset = false;
        }
    }


    /**
     * this func normalize the angle to be between -pi to pi
     *
     * @param angle the angle to normalize
     * @return the normalized angle
     */
    public double NormalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= TPI;
        }
        while (angle < -Math.PI) {
            angle += TPI;
        }
        return angle;
    }

    /**
     * this function will return the heading of the robot (ccw is positive) in radians (0 to 2pi) and make sure that the start R is taken into account
     *
     * @return
     */
    public double Heading() {
        //return the heading of the robot (ccw is positive) in radians (0 to 2pi) and make sure that the start R is taken into account
        robotHading_CWP = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + startR - reset; //cw is positive
        robotHading_CCWP = -robotHading_CWP; //ccw is positive
        return NormalizeAngle(robotHading_CCWP); //normalize the angle to be between -pi and pi
    }
    public void setWantedAngle(){
        wantedAngle = Heading();
    }


    /**
     * this function will allow the robot to move in any direction
     *
     * @param x              forward and backward movement
     * @param y              left and right movement
     * @param r              rotation
     * @param isFieldCentric if the robot is field centric or not
     */
    public void drive(double x, double y, double r, boolean isFieldCentric, boolean lookR) {
        //drive_thread the robot X is forward and backward, Y is left and right, R is rotation'
        update();
        double rotX = 0;
        double rotY = 0;
        double newR =0;

        if(lookR){
            //wantedAngle += r*13;
            newR = rPid.calculate (NormalizeAngle(wantedAngle  - Heading()));
        }
        else {
            newR = r;
        }

        if (isFieldCentric) {
            double botHeading = Heading();
            rotX = x * cos(botHeading) - y * sin(botHeading);
            rotY = x * sin(botHeading) + y * cos(botHeading);
        } else {
            //if the robot is not field centric then the robot will move in the direction of the robot
            rotX = x;
            rotY = y;
        }

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(newR), 1);
        double frontLeftPower = (rotY + rotX + newR) / denominator;
        double backLeftPower = (rotY - rotX + newR) / denominator;
        double frontRightPower = (rotY - rotX - newR) / denominator;
        double backRightPower = (rotY + rotX - newR) / denominator;



        //set the power of the motors
        flm.setPower(frontLeftPower);
        frm.setPower(frontRightPower);
        blm.setPower(backLeftPower);
        brm.setPower(backRightPower);


    }

    /**
     * update the field position of the robot using the encoders and the imu (gyro) sensor to calculate the position of the robot on the field using the odometry
     */
    private void update() {
        //save the Encoder position
        double leftEncoderPos = getXlEncoder();
        double rightEncoderPos = getXrEncoder();
        double centerEncoderPos = getYEncoder();

        //calculate the change in encoder position from the previous iteration of the loop
        double deltaLeftEncoderPos = leftEncoderPos - prevLeftEncoderPos;
        double deltaRightEncoderPos = rightEncoderPos - prevRightEncoderPos;
        double deltaCenterEncoderPos = centerEncoderPos - prevCenterEncoderPos;

        //calculate the change in position of the robot
        double phi = (deltaLeftEncoderPos - deltaRightEncoderPos) / LATERAL_DISTANCE;
        double deltaMiddlePos = (deltaLeftEncoderPos + deltaRightEncoderPos) / 2;
        double deltaPerpPos = deltaCenterEncoderPos - FORWARD_OFFSET * phi;

        //calculate the change in the field position of the robot
        double heading = robotHading_CCWP;
        double deltaX = deltaMiddlePos * cos(heading) + deltaPerpPos * sin(heading);
        double deltaY = -deltaMiddlePos * sin(heading) + deltaPerpPos * cos(heading);

        //update the field position of the robot
        fieldX += ticksToMM(deltaX);
        fieldY += ticksToMM(deltaY);
        Heading();

        //save the encoder position for the next iteration of the loop
        prevLeftEncoderPos = leftEncoderPos;
        prevRightEncoderPos = rightEncoderPos;
        prevCenterEncoderPos = centerEncoderPos;
        opMode.telemetry.update();

    }
    public void  fieldToRobotConvert(double deltaX ,double deltaY) {
        //convert the  field deltas to robot deltas

        double robotDeltaX = deltaX * Math.cos(Heading()) - deltaY * Math.sin(Heading());
        double robotDeltaY = deltaX * Math.sin(Heading()) + deltaY * Math.cos(Heading());

        errors[0] = robotDeltaX;
        errors[1] = robotDeltaY;
    }

    /**
     * drive_thread to a certain position on the field with a certain rotation (r) in degrees
     *
     * @param x the x position on the field
     * @param y the y position on the field
     * @param r the rotation of the robot in degrees
     */

    public boolean driveTo(double x, double y, double r) {
        //the default timeout is 5 seconds
        return driveTo(x, y, r, 3000);
    }
    public boolean driveTo(double x, double y, double r, double timeOut) {


            double xPower = 0;
            double yPower = 0;
            double rPower = 0;
            //drive_thread to a certain position
            //reset the timer
            double startTime = time.milliseconds();
            do {
                update();
                //check if the robot has tried for more than timeOut milliseconds
                if (time.milliseconds() - startTime > timeOut ) {
                    //stop the robot
                    drive(0, 0, 0, false,false);
                    //return false
                    opMode.telemetry.clearAll();
                    opMode.telemetry.addData("timeOut", "timeOut");

                    return false;
                }

                //calculate the error in the position
                fieldToRobotConvert(x - getFieldX(), y - getFieldY());
                //calculate the power needed to get to the position
                xPower = xPid.calculate(errors[0]);
                yPower = yPid.calculate(errors[1]);
                rPower = rPid.calculate((Math.toRadians(r)- Heading()));
                //limit the power to 0.7
                xPower = Range.clip(xPower, -0.7, 0.7);
                yPower = Range.clip(yPower, -0.7, 0.7);
                rPower = Range.clip(rPower, -0.7, 0.7);
               // opMode.telemetry.addData("x", fieldX);
               // opMode.telemetry.addData("y", fieldY);
               // opMode.telemetry.addData("r", Math.toDegrees(Heading()));
                //print the errors
               // opMode.telemetry.addData("x error", errors[0]);
               // opMode.telemetry.addData("y error", errors[1]);
                opMode.telemetry.addData("x Pow",xPower);
                opMode.telemetry.addData("fx",fieldX);

               // opMode.telemetry.addData("y Pow",yPower);
               // opMode.telemetry.addData("r Pow",rPower);




                //drive the robot to the position with the calculated power and the robot is field centric

                drive(-yPower,xPower, rPower, false,false);

            } while ((xPower != 0 || yPower != 0 || rPower != 0) && opMode.opModeIsActive() && !opMode.isStopRequested());//if the robot is at the position (or the op mode is off) then stop the loop
            //stop the robot
            drive(0, 0, 0, false,false);
            //return true if the robot is at the position
            return true;
    }

    /**
     * this method will change the mode of the robot from field centric to not field centric and vice versa
     *
     * @param x the state of the button that is being used to change the mode
     */
    public void changeMode(boolean x) {
        //change the mode of the robot
        if (!wasChanged && x) {
            //if the robot is not in the mode that is being changed to then change the mode
            wasChanged = true;
            //change the mode of the robot if it was field centric then it is not and if it was not then it is
            field = !field;
        } else if (!x) {
            wasChanged = false;
        }
        opMode.telemetry.addData("filed: ", field);
    }
}














