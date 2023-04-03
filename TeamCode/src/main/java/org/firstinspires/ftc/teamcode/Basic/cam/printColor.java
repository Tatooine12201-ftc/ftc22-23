package org.firstinspires.ftc.teamcode.Basic.cam;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Basic.cam.ColorCamera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Basic.cam.zone;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

    @TeleOp(name = "printColor")
    public class printColor extends LinearOpMode {
        // public ElapsedTime runtime = new ElapsedTime();
        OpenCvWebcam webcam;
        zone result;
        ColorCamera colorCamera;
        int cameraMonitorViewId;

        @Override
        public void runOpMode() {

            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            colorCamera = new ColorCamera(telemetry);
            webcam.setPipeline(colorCamera);
            webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                }
            });
            result = colorCamera.getResult();
            webcam.stopStreaming();
            waitForStart();


            if (result == zone.A) {
                telemetry.addData("yellow" ,zone.A);

            } else if (result == zone.B) {
                telemetry.addData("green" ,zone.B);

            } else {
                telemetry.addData("no " ,zone.C);
            }

        }
    }


