package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Basic.AprilTagCamera.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "rightSide1")
public class rightSide1 extends ThreadOpMode {

    Mecanum mecanum;
    lift lift;
    Fourbar fourbar;
    Pliers pliers;

    static final double FEET_PER_METER = 3.28084;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
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


    private boolean isRuning() {
        return opModeIsActive() && !isStopRequested();
    }

    @Override
    public void mainInit() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        mecanum = new Mecanum(hardwareMap, this);

        camera.setPipeline(aprilTagDetectionPipeline);
        mecanum.setStartPos(0, 0, 0);
        lift = new lift(hardwareMap, this);

        pliers = new Pliers(hardwareMap);

        fourbar = new Fourbar(hardwareMap, this);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void subsystemLoop() {
                fourbar.spin(0);
            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void subsystemLoop() {
                lift.move(0);
            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void subsystemLoop() {
               // mecanum.drive(0,0,0,false,false);
            }
        }));

    }
    @Override
    public void mainLoop() {



    }

    @Override
    public void runOpMode() throws InterruptedException
    {
        mainInit();
        waitForStart();
        startThreads();
        pliers.Open();
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        mecanum.reset();
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        /*while (isRuning())
        {
            telemetry.addData("x:", );
            }
        */

        boolean liftDone = false;
        boolean fourBarDone = false;
        lift.setLevel(3);
        fourbar.setLevel(2);
        mecanum.driveTo(1325,40,0,10000);
        sleep(1000);

     //   while (!lift.isatpos()){
         //   lift.setLevel(lift.autoHige);
       // }

      // sleep(100000);
      // while (fourbar.isatpos()){
           //fourbar.setLevel(2);
      // }

       // mecanum.driveTo(1280,60,0,1000);
        pliers.close();
        sleep(200);
        pliers.Open();
        sleep(200);
        sleep(1000000);

       // mecanum.driveTo(1515,70,0,100);

       // mecanum.driveTo(1225,100,0,500);

       // while (fourbar.isatpos()){
          //  fourbar.setLevel(0);
       // }

       // while (lift.isatpos()){
           // lift.setLevel(0);
      //  }

       // pliers.Open();
       // mecanum.driveTo(1250,580,0,100);
       // mecanum.driveTo(300,0,0,10000);
      /*  mecanum.driveTo(1158, 62, 0, 1500);

        lift.setLevel(lift.autoHige);
        while (!liftDone && isRuning()) {
            liftDone = lift.move(0);
        }
        liftDone = false;
        fourbar.setLevel(2);
        while (!fourBarDone && isRuning()) {
            lift.move(0);
            //fourBarDone = fourbar.spin(0);
        }
        fourBarDone = false;
        mecanum.driveTo(1280, 60, 0, 2000);
        telemetry.addData("fD", fourBarDone);
        pliers.Open();
        sleep(200);
        pliers.close();
        sleep(200);

        //   mecanum.driveTo(1200, 35, 0,500);
        mecanum.driveTo(1225, 100, 0, 500);
        fourbar.setLevel(0);
        while (!fourBarDone && isRuning()) {
            lift.move(0);
            //fourBarDone = fourbar.spin(0);
        }
        fourBarDone = false;

        lift.setLevel(lift.abouveWheel);
        fourbar.setLevel(0);
        while (!liftDone && isRuning()) {
            fourbar.spin(0);
            liftDone = lift.move(0);
        }
        liftDone = false;

        /**second cycle*/

//        mecanum.driveTo(1200, 30, 90,1000);
//
//        pliers.Open();
//        sleep(200);
//
//        fourbar.setLevel(0);
//        while (!fourBarDone && isRuning()) {
//            lift.move(0);
//            fourBarDone = fourbar.spin(0);
//        }
//        fourBarDone = false;
//
//        mecanum.driveTo(1350, -500, 90,1500);
//
//        lift.setLevel(lift.autoStack4);
//        while (!liftDone && isRuning()) {
//            fourbar.spin(0);
//            liftDone = lift.move(0);
//        }
//        liftDone = false;
//
//        fourbar.setLevel(0);
//        while (!fourBarDone && isRuning()) {
//            lift.move(0);
//            fourBarDone = fourbar.spin(0);
//        }
//        fourBarDone = false;
//
//        mecanum.driveTo(1350, -590, 90,2000);
//
//        lift.setLevel(lift.autoStack4);
//
//        while (!liftDone && isRuning()) {
//            fourbar.spin(0);
//            liftDone = lift.move(0);
//        }
//        liftDone = false;
//
//        fourbar.setLevel(0);
//        while (!fourBarDone && isRuning()) {
//            lift.move(0);
//            fourBarDone = fourbar.spin(0);
//        }
//        fourBarDone = false;
//
//        pliers.close();
//        sleep(600);
//
//        lift.setLevel(lift.liftStack);
//        while (!liftDone && isRuning()) {
//            fourbar.spin(0);
//            liftDone = lift.move(0);
//        }
//        liftDone = false;
//
//        fourbar.setLevel(0);
//        while (!fourBarDone && isRuning()) {
//            lift.move(0);
//            fourBarDone = fourbar.spin(0);
//        }
//        fourBarDone = false;
//
//        mecanum.driveTo(1250, 320, 90,2000);
//
//        lift.setLevel(lift.autoHige);
//        while (!liftDone && isRuning()) {
//            liftDone = lift.move(0);
//        }
//        liftDone = false;
//        fourbar.setLevel(2);
//        while (!fourBarDone && isRuning()) {
//            lift.move(0);
//            fourBarDone = fourbar.spin(0);
//        }
//        fourBarDone = false;
//
//        mecanum.driveTo(1310, 540, 90);
//
//        sleep(1000);
//
//        pliers.Open();
//
//        sleep(300);

 /*       pliers.close();
        mecanum.driveTo(1250, 580, 0, 2000);

        sleep(1000);

        /** 3 cycel */

   //     /** park*/
   /*     fourbar.setLevel(0);
        while (!fourBarDone && isRuning()) {
            //fourBarDone = fourbar.spin(0);
        }
        fourBarDone = false;

        lift.setLevel(0);
        while (!liftDone && isRuning()) {
            fourbar.spin(0);
            liftDone = lift.move(0);
        }
        liftDone = false;
        pliers.Open();

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {


            mecanum.driveTo(1200, 580, 0, 5000);


        } else if (tagOfInterest.id == MIDDLE) {

            mecanum.driveTo(1200, 10, 0, 5000);

        } else {
            mecanum.driveTo(1200, -630, 0, 5000);


        }
        pliers.Open();

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */

        stopThreads();
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

    // }
    /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
    //   while (opModeIsActive()) {sleep(20);}
    ///other code insted
    // }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("Detected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}





