package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.Locale;

@Autonomous(name = "CVTest", group="Test")

public class CVTest extends LinearOpMode {

    private OpenCvCamera webcam;
    private YellowDetector yellowDetector;
    private MarkerDetector markerDetector;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.openCameraDevice();

        yellowDetector = new YellowDetector();
        markerDetector = new MarkerDetector();

        webcam.setPipeline(yellowDetector);

        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive())
        {

            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Detected State: ", yellowDetector.isDetected);
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format(Locale.US, "%.2f", webcam.getFps()));
            telemetry.update();
        }

        webcam.closeCameraDevice();
    }

}