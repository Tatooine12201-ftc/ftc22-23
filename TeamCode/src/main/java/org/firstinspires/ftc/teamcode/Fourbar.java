package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Fourbar {

    private static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder

    private static final double DRIVE_GEAR_REDUCTION = 56.0 / 30.0;     // This is < 1.0 if geared UP
    private static final double tiksPerDegree = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;

    private static final double SERVO_DEGREE_TO_FOURBAR_DEGREE = DRIVE_GEAR_REDUCTION * 360;
    // public static DcMotor rightFourbar = null;
    // public static DcMotor leftFourbar = null;
    public static Servo rightServo = null;
    public static Servo leftServo = null;
    private static final boolean isBusy = false;
    Pid pid;
    LinearOpMode opMode;
    private int level = 0;
    // private final int[] levels = {0,138,-138,180} ;//140
    private final double[] levels = {0, 0.5 ,1 };//140
    private boolean manual = false;

    private static double offset = 0;
    private double Fourbar_speed = 0;
    private double Fourbar_pos = 0;
    int prevLevel = 0;
    double F = 0;

    public Fourbar(HardwareMap hw, LinearOpMode opMode) {
        this.opMode = opMode;
        //  rightFourbar = hw.get(DcMotor.class, "rightFourbar");
        //  rightFourbar.setDirection(REVERSE);

        //  leftFourbar = hw.get(DcMotor.class, "leftFourbar");
        // leftFourbar.setDirection(REVERSE);

        rightServo = hw.get(Servo.class, "rightServo");
        rightServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setPosition(0.5);


        leftServo = hw.get(Servo.class, "leftServo");
        leftServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setPosition(0.5);

        //leftFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rightFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // leftFourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //  rightFourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //pid = new Pid( 0.016953,0.0001, 0, 0.08, true);
        // F = pid.getF();
        // pid.setMaxIntegral(0.1);
        // pid.setTolerates(3);


    }


    public void resetservo() {
        //  leftFourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  rightFourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //  leftFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rightFourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        offset = 0;
        manual = false;
        level = 0;
    }

    public void reset() {
        resetservo();
    }


    public void setManual() {
        manual = true;
    }

    public void setLevel(int level) {
        if (level >= 0 && level <= 2) {
            this.level = level;
        }
    }


    public void spin(double m) {
        double turget = levels[level];
        //  double err = turget - rightServo.getPosition();
        if (opMode.opModeIsActive() && !opMode.isStopRequested()) {


            rightServo.setPosition(turget);
            leftServo.setPosition(turget);


        }


    }
}









