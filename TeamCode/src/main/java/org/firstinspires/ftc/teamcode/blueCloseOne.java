package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Basic.cam.ColorCamera;
import org.firstinspires.ftc.teamcode.Basic.cam.zone;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


public class blueCloseOne {
    @Autonomous(name = "blueCloseOne",group = "auto")
    public class bluecloseone extends LinearOpMode {
       public ElapsedTime runtime = new ElapsedTime();



        OpenCvWebcam webcam;
        zone result;
        ColorCamera colorCamera;
        int cameraMonitorViewId;



        @Override
        public void runOpMode() {

           Mecanum mecanum = new Mecanum(hardwareMap, this);
           mecanum.resetEncoders();

            lift lift = new lift(hardwareMap, this);
           Pliers pliers = new Pliers(hardwareMap);
           pliers.close();

            Fourbar fourbar = new Fourbar(hardwareMap, this);
            cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            colorCamera = new ColorCamera(telemetry);
            webcam.setPipeline(colorCamera);

            webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {
                }
            });


            result = colorCamera.getResult();
            webcam.stopStreaming();
            mecanum.setStartingPoint(0 ,900 ,0);
            waitForStart();
            runtime.reset();
           mecanum.driveTo(1000,0.3,0);
           lift.setLevel(3);
            lift.move();
            fourbar.setLevel(1);
              fourbar.spin(0);
              fourbar.setLevel(0);
               lift.setLevel(0);
              mecanum.driveTo(1000,0,90);


              if(result == zone.A){

              }
              else if(result==zone.B){

              }
              else {

              }












        }

      }
  }
