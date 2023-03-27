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
    boolean mecanumDone = false;

    lift lift;
    boolean liftDone = false;

    Fourbar fourbar;
    boolean fourbarDone = false;
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

    private boolean shouldPark() {
        return timer.seconds() > 28;
    }

    ElapsedTime timer = new ElapsedTime();

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
                fourbarDone = fourbar.spin(0);
            }
        }));

        registerThread(new TaskThread(new TaskThread.Actions() {
            @Override
            public void subsystemLoop() {
                liftDone = lift.move(0);
            }
        }));


    }

    @Override
    public void mainLoop() {


    }

    @Override
    public void runOpMode() throws InterruptedException {
        mainInit();
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
        startThreads();



        /*while (isRuning())
        {
            telemetry.addData("x:", );
            }
        */

        boolean liftDone = false;
        boolean fourBarDone = false;

        timer.reset();

        lift.setLevel(4);
        fourbar.setLevel(2);
        mecanum.driveTo(1335, 50, 0, 3000);
        pliers.close();
        sleep(200);
        pliers.Open();
        sleep(200);
        mecanum.driveTo(980, 0, 0, 1000);
        for (int i = 0; i < 1; i++) {
            fourbar.setLevel(0);
            sleep(100);

            if (i == 4) {
                lift.setLevel(0);
            } else {
                lift.setLevel(i + 5);
            }

            mecanumDone = mecanum.driveTo(1320, 0, -90, 1000);


            pliers.close();
            sleep(200);
            mecanum.driveTo(1130, 810, -90, 1000);
            pliers.Open();
            sleep(500);
            lift.setLevel(lift.autoHige);
            sleep(500);
            mecanum.driveTo(1130, 40, -90, 1000);
            sleep(100);
            fourbar.setLevel(2);
            mecanum.driveTo(1300,-50,-90);


            pliers.close();
            sleep(200);
            mecanum.driveTo(1130, 810, -90, 1000);

        }

        /**park**/

        // fourbar.setLevel(0);
        // lift.setLevel(0);
        //  mecanum.driveTo(1250,0,0,1500);
        // if (tagOfInterest==null||tagOfInterest.id==LEFT){
        //   mecanum.driveTo(1200,580,500);



           // else if (tagOfInterest.id == MIDDLE) {
          //  mecanum.driveTo(1200, 10, 0);

      //  } else {
           // mecanum.driveTo(1200, -630, 0);
      //  }

        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available ,it was never during the init loop :(");
            telemetry.update();
        }



        stopThreads();
        telemetry.addData("forbar ", fourbar.getEncoder());
        telemetry.update();

    }





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





