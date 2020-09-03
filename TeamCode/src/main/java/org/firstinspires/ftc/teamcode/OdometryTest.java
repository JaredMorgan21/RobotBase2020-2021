package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Odometry Test", group="Test")
public class OdometryTest extends LinearOpMode {
    private Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();
        if(opModeIsActive()){

            //forward until 100 ticks
            robot.forward(1);
            while(opModeIsActive() && robot.getY() < 100){
                telemetry.addData("Y: ", robot.getY());
                telemetry.update();
            }
            robot.stop();

            //right until 100 ticks
            robot.sideways(1);
            while(opModeIsActive() && robot.getX() < 100){
                telemetry.addData("X: ", robot.getX());
                telemetry.update();
        }
            robot.stop();

            //diagonal until 100 ticks each
            robot.diagonal(1);
            while(opModeIsActive() && robot.getX() < 200 && robot.getY() < 200){
                telemetry.addData("Y: ", robot.getY());
                telemetry.addData("X: ", robot.getX());
                telemetry.update();
            }
            robot.stop();

            robot.go2Point(12, 12);

            //turn until 90 ticks
            robot.turn(1);
            while(opModeIsActive() && robot.getAngleOdoDegrees() < 90){
                telemetry.addData("Rot: ", robot.getAngleOdoDegrees());
                telemetry.update();
            }
            robot.stop();
        }
    }
}
