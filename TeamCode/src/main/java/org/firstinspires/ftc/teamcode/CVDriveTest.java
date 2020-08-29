package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

public class CVDriveTest extends LinearOpMode {
    private Hardware robot = new Hardware();

    private OpenCvCamera phoneCam;
    private YellowDetector yellowDetector;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        phoneCam.openCameraDevice();

        yellowDetector = new YellowDetector();
        phoneCam.setPipeline(yellowDetector);

        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

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
