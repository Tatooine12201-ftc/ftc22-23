package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.CompletableFuture;


@TeleOp(name = "Tests")
public class Tests extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Mecanum mecanum = new Mecanum(hardwareMap, this);
        lift lift = new lift(hardwareMap, this);
        Fourbar fourbar = new Fourbar(hardwareMap, this);

        mecanum.reset();
        mecanum.setStartPos(0,0,0);

        waitForStart();
        //pid tuner
        double kP = 0;
        double kI = 0;
        double kD = 0;
        double kF = 0;
        double maxI = 0;

        //find the min value the drivetrain can move and save it to maxI (for x,y and rotation)
        boolean found = false;
        while(!found){
            mecanum.drive(maxI,0,0,false);
            sleep(100);
            if(mecanum.getFieldX() > 2){
                found = true;
            }
            else {
                maxI += 0.01;
            }
        }
        mecanum.drive(0,0,0,false);
        double XmaxI = maxI;
        telemetry.addData("x", XmaxI);
        maxI = 0;
        found = false;
        while (!found){
            mecanum.drive(0,maxI,0,false);
            sleep(100);
            if(mecanum.getFieldY() > 2){
                found = true;
            }
            else {
                maxI += 0.01;
            }
        }
        mecanum.drive(0,0,0,false);
        double YmaxI = maxI;
        telemetry.addData("y", YmaxI);
        maxI = 0;
        found = false;
        while (!found){
            mecanum.drive(0,0,maxI,false);
            sleep(100);
            if(Math.toDegrees(mecanum.Heading()) > 2){
                found = true;
            }
            else {
                maxI += 0.01;
            }
        }
        mecanum.drive(0,0,0,false);
        double RmaxI = maxI;
        telemetry.addData("r", RmaxI);
        telemetry.update();
        sleep(10000);
        //do a SYSid test for the drivetrain and save the values to kP, kI, kD, kF


    }
}
