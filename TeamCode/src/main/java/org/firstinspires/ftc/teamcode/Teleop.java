package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Basic Teleop")
public class Teleop extends LinearOpMode {
    private DcMotor LeftFrontMotor = null;
    private DcMotor LeftBackMotor = null;
    private DcMotor RightFrontMotor = null;
    private DcMotor RightBackMotor = null;
    private DcMotor LiftMotor = null;
    private DcMotor ForBarMotor = null;
    private Servo GripperServo  = null;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        //הנעה
        LeftFrontMotor = hardwareMap.get(DcMotor.class,"left front motor");
        LeftBackMotor = hardwareMap.get(DcMotor.class,"left back motor");
        RightFrontMotor = hardwareMap.get(DcMotor.class,"right front motor");
        RightBackMotor = hardwareMap.get(DcMotor.class,"right back motor");

        LiftMotor = hardwareMap.get(DcMotor.class,"lift motor");
        ForBarMotor = hardwareMap.get(DcMotor.class,"forbar motor");
        GripperServo = hardwareMap.get(Servo.class,"gripper servo");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);
        waitForStart();

    }
}
