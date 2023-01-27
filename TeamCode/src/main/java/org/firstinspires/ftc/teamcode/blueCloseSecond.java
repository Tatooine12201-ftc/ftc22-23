package org.firstinspires.ftc.teamcode;

import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Basic.AprilTagCamera.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;

@Autonomous
    public class blueCloseSecond extends LinearOpMode
    {
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        // UNITS ARE METERS
        double tagsize = 0.166;

        // Tag ID 1,2,3 from the 36h11 family
        int LEFT = 1;
        int MIDDLE = 2;
        int RIGHT = 3;

        AprilTagDetection tagOfInterest = null;
        CompletableFuture<Void> drive_thread;
        CompletableFuture<Void> gripper_thread;
        CompletableFuture<Void> lift_thread;

        private boolean isRuning(){
            return opModeIsActive() && !isStopRequested();
        }
        @Override
        public void runOpMode()
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
            Mecanum mecanum = new Mecanum(hardwareMap, this);

            camera.setPipeline(aprilTagDetectionPipeline);
            mecanum.setStartPos(0,0,0);
            lift lift = new lift(hardwareMap, this);

            Pliers pliers = new Pliers(hardwareMap);

            Fourbar fourbar = new Fourbar(hardwareMap, this);



            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });

            telemetry.setMsTransmissionInterval(50);
            pliers.close();
            /*
             * The INIT-loop:
             * This REPLACES waitForStart!
             */
            while (!isStarted() && !isStopRequested())
            {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

                if(currentDetections.size() != 0)
                {
                    boolean tagFound = false;

                    for(AprilTagDetection tag : currentDetections)
                    {
                        if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                        {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }

                    if(tagFound)
                    {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    }
                    else
                    {
                        telemetry.addLine("Don't see tag of interest :(");

                        if(tagOfInterest == null)
                        {
                            telemetry.addLine("(The tag has never been seen)");
                        }
                        else
                        {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }

                }

                telemetry.update();
                sleep(20);
            }
            mecanum.reset();
            lift.reset();
            fourbar.reset();

            ElapsedTime timer =new ElapsedTime();
            timer.reset();
            pliers.close();
            sleep(100);


            if (isRuning()){
            drive_thread = mecanum.driveTo(735,80,0);}


            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && isRuning()) {
                lift_thread = CompletableFuture.runAsync(() -> {
                    lift.setLevel(lift.autoMid);
                    boolean liftDone = false;
                    do {
                        liftDone = lift.move(0);
                    }
                    while (!liftDone && isRuning());

                    fourbar.setLevel(1);

                    boolean fourbarDone = false;
                    do {
                        fourbarDone = fourbar.spin(0);
                    }
                    while (!fourbarDone && isRuning());
                });
            }
            while(!drive_thread.isDone() && isRuning()) {
                telemetry.addData("running",true);
                telemetry.update();
            }

            pliers.Open();
            sleep(100);
            if (isRuning()){
            drive_thread = mecanum.driveTo(1250,0,90);}
            while(!drive_thread.isDone()  && isRuning()) {
                telemetry.addData("running",true);
                telemetry.update();
            }
            pliers.close();
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && isRuning()) {
                lift_thread = CompletableFuture.runAsync(() -> {

                    fourbar.setLevel(0);
                    boolean fourbarDone = false;
                    do {
                        fourbarDone = fourbar.spin(0);
                    }
                    while (!fourbarDone && isRuning());


                });
            }
            while(!lift_thread.isDone() && isRuning()) {
                telemetry.addData("running",true);
                telemetry.update();
            }

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && isRuning()) {
                lift_thread = CompletableFuture.runAsync(() -> {
                    lift.setLevel(lift.autoStack4);
                    boolean liftDone = false;
                    do {
                        liftDone = lift.move(0);
                    }
                    while (!liftDone && isRuning());
                    fourbar.setLevel(0);
                    boolean fourbarDone = false;
                    do {
                        fourbarDone = fourbar.spin(0);
                    }
                    while (!fourbarDone && isRuning());
                });
            }
            pliers.Open();
            sleep(300);
            drive_thread = mecanum.driveTo(1250,-480,90);


            while(!drive_thread.isDone() && !lift_thread.isDone() && isRuning()) {
                telemetry.addData("running",true);
                telemetry.update();
            }
            sleep(500);
            pliers.close();
            sleep(500);

            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N && isRuning()) {
                lift_thread = CompletableFuture.runAsync(() -> {
                    lift.setLevel(2);
                    boolean liftDone = false;
                    do {
                        liftDone = lift.move(0);
                    }
                    while (!liftDone && isRuning());
                    fourbar.setLevel(0);
                    boolean fourbarDone = false;
                    do {
                        fourbarDone = fourbar.spin(0);
                    }
                    while (!fourbarDone && isRuning());
                });
            }
            while(!drive_thread.isDone() && !lift_thread.isDone() && isRuning()) {
                telemetry.addData("running",true);
                telemetry.update();
            }














            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            /* Update the telemetry */
            if(tagOfInterest != null)
            {
                telemetry.addLine("Tag snapshot:\n");
                tagToTelemetry(tagOfInterest);
                telemetry.update();
            }
            else
            {
                telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
                telemetry.update();
            }

            /* Actually do something useful */

          /// if(tagOfInterest == null || tagOfInterest.id == LEFT) {
               // mecanum.driveTo(700,0, 0);
                 //mecanum.driveTo(700,515, 0);
                // mecanum.driveTo(800,515, 0);

            // }else if(tagOfInterest.id == MIDDLE){
               // mecanum.driveTo(905,0, 0);
             //}else{
               //  mecanum.driveTo(720,0, 0);
                //  mecanum.driveTo(700,-515, 0);
                // mecanum.driveTo(730,-515, 0);

             }
            /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
         //   while (opModeIsActive()) {sleep(20);}
            ///other code insted
       // }

        void tagToTelemetry(AprilTagDetection detection)
        {
            telemetry.addLine(String.format("Detected tag ID=%d", detection.id));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
            telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }
    }


