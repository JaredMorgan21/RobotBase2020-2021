package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Motor Test", group="Test")
public class MotorTest extends LinearOpMode {
    private Hardware robot = new Hardware(this);

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        waitForStart();
        if(!isStopRequested()){

            robot.forward(1);
            sleep(1000);
            robot.stop();

            robot.sideways(1);
            sleep(1000);
            robot.stop();

            robot.diagonal(1);
            sleep(1000);
            robot.stop();
        }
    }
}
