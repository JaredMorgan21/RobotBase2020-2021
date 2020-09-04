package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;

@Autonomous(name = "CVTest", group="Test")

public class CVTest extends LinearOpMode {

    private OpenCvCamera phoneCam;
    private YellowDetector yellowDetector;
    private MarkerDetector markerDetector;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        yellowDetector = new YellowDetector();
        markerDetector = new MarkerDetector();

        phoneCam.setPipeline(markerDetector);

        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive())
        {

            /*
             * Send some stats to the telemetry
             */

//            if(yellowDetector.isDetected){
//
//            }

            telemetry.addData("Detected State: ", yellowDetector.isDetected);
            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", phoneCam.getFps()));
            telemetry.update();
        }

        phoneCam.closeCameraDevice();
    }

}