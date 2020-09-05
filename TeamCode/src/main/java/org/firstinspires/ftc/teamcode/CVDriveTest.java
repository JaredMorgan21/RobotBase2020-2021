package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="CV Drive Test", group="Test")
public class CVDriveTest extends LinearOpMode {
    private Hardware robot = new Hardware();

    private OpenCvCamera webcam;
    private YellowDetector yellowDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        webcam.openCameraDevice();

        yellowDetector = new YellowDetector();
        webcam.setPipeline(yellowDetector);

        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

        waitForStart();
        if(opModeIsActive()){
            robot.sideways(1);

            while(!yellowDetector.isDetected){
                sleep(1);
            }
            robot.stop();


        }
    }
}
