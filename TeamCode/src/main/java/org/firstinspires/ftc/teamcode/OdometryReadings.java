package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Odometry Readings", group="Test")
public class OdometryReadings extends LinearOpMode {
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("RotationIMU: ", robot.getAngleIMU());
            telemetry.addData("RotationOdo: ", robot.getAngleOdoDegrees());
            telemetry.addData("X: ", robot.getX());
            telemetry.addData("Y: ", robot.getY());
            telemetry.addData("Left: ", robot.getLeft());
            telemetry.addData("Right: ", robot.getRight());
            telemetry.update();
        }
    }
}