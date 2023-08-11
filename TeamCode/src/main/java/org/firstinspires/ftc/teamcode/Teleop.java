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


    private DriveTrain driveTrain;
    private Lift lift;
    private Gripper gripper;
    private FourBar fourBar;

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


        driveTrain = new DriveTrain(LeftFrontMotor,LeftBackMotor,RightFrontMotor,RightBackMotor,imu);
        lift = new Lift(LiftMotor);
        gripper = new Gripper(GripperServo);
        fourBar = new FourBar(ForBarMotor);
        waitForStart();
        while (opModeIsActive()){
            driveTrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);
            lift.lift(-gamepad2.left_stick_y);
            if (gamepad2.a){
                gripper.open();
            }
            else if (gamepad2.b){
                gripper.close();
            }

        }

    }
}
