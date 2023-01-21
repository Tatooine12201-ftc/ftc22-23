package org.firstinspires.ftc.teamcode.Basic;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Basic.cam.ColorCamera;
import org.firstinspires.ftc.teamcode.Basic.cam.zone;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "cam test")
public class Auto extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ColorCamera colorCamera = new ColorCamera(telemetry);
        webcam.setPipeline(colorCamera);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            //   phoneCam.openCamerDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()


            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            //   telemetry.addLine("waiting for start");
            //   telemetry.update();


            @Override
            public void onError(int errorCode) {
            }
        });
        waitForStart();
        zone result = colorCamera.getResult();
        webcam.stopStreaming();
        switch (result) {
            case A:
                break;
            case B:
                break;
            case C:
                break;
        }
    }

}
