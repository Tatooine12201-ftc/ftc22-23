package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

public class DriveTrain {


    private IMU imu;
    DcMotor motorFrontLeft  = null;
        DcMotor motorBackLeft   = null;
        DcMotor motorFrontRight = null;
        DcMotor motorBackRight  = null;

    public DriveTrain(DcMotor motorFrontLeft, DcMotor motorBackLeft, DcMotor motorFrontRight, DcMotor motorBackRight, IMU imu) {

        this.motorFrontLeft = motorFrontLeft;
        this.motorBackLeft = motorBackLeft;
        this.motorFrontRight = motorFrontRight;
        this.motorBackRight = motorBackRight;
        this.imu = imu;
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public DcMotor getMotorFrontLeft() {
        return motorFrontLeft;
    }

    public void setMotorFrontLeft(DcMotor motorFrontLeft) {
        this.motorFrontLeft = motorFrontLeft;
    }

    public DcMotor getMotorBackLeft() {
        return motorBackLeft;
    }

    public void setMotorBackLeft(DcMotor motorBackLeft) {
        this.motorBackLeft = motorBackLeft;
    }

    public DcMotor getMotorFrontRight() {
        return motorFrontRight;
    }

    public void setMotorFrontRight(DcMotor motorFrontRight) {
        this.motorFrontRight = motorFrontRight;
    }

    public DcMotor getMotorBackRight() {
        return motorBackRight;
    }

    public void setMotorBackRight(DcMotor motorBackRight) {
        this.motorBackRight = motorBackRight;
    }

    public IMU getImu() {
        return imu;
    }

    public void setImu(IMU imu) {
        this.imu = imu;
    }




        public void drive(double y,double x,double rx) {

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;



            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);


        }
}
