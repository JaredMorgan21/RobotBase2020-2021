package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {
    double globalAngle;
    double currentAngle;

    public DcMotor BLM;
    public DcMotor BRM;
    public DcMotor FLM;
    public DcMotor FRM;
    public DcMotor flywheelL;
    public DcMotor flywheelR;

    public DcMotor encoderX;
    public DcMotor encoderRight;
    public DcMotor encoderLeft;

    public Blinker expansion_Hub_2;

    public TouchSensor touch;

    public ElapsedTime RunTime = new ElapsedTime();

    public Servo claw;
    public Servo push;
    public CRServo pulley;

    BNO055IMU Imu;
    public Orientation prevAngles = new Orientation();
    public Gyroscope imu;

    public double WHEEL_DIAMETER = 1.37795;
    public double rotWheelDistance = 2;
    public Hardware(){

    }

    public void init(HardwareMap hardwareMap){
        //Getting Motor Name From Phone
        BLM = hardwareMap.dcMotor.get("BLM");
        BRM = hardwareMap.dcMotor.get("BRM");
        FLM = hardwareMap.dcMotor.get("FLM");
        FRM = hardwareMap.dcMotor.get("FRM");
        flywheelL = hardwareMap.dcMotor.get("flywheelL");
        flywheelR = hardwareMap.dcMotor.get("flywheelR");

        encoderX = hardwareMap.dcMotor.get("encoderX");
        encoderRight = hardwareMap.dcMotor.get("encoderRight");
        encoderLeft = hardwareMap.dcMotor.get("encoderLeft");

        // showing which direction the robot moves
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);

        // getting servos from phone
        claw = hardwareMap.servo.get("claw");
        push = hardwareMap.servo.get("push");
        pulley = hardwareMap.crservo.get("pulley");
        touch = hardwareMap.touchSensor.get ("touch");

        // setting parameters for gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        // setting to run as an inertial measurement unit
        parameters.mode = BNO055IMU.SensorMode.IMU;

        // set to meausure in degrees
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled = false;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        // getting imu name from phone
        Imu = hardwareMap.get(BNO055IMU.class, "imu");
        Imu.initialize(parameters);
    }

    public int getRight(){
        return encoderRight.getCurrentPosition();
    }

    public int getLeft(){
        return encoderLeft.getCurrentPosition();
    }

    public int getX(){
        return encoderX.getCurrentPosition();
    }

    public int getY(){
        return (getRight() + getLeft())/2;
    }

    public void forward(double power){
        BLM.setPower(power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(power);
    }

    public void sideways(double power){
        BLM.setPower(-power);
        BRM.setPower(power);
        FLM.setPower(power);
        FRM.setPower(-power);
    }

    public void diagonal(double power){
        FLM.setPower(power);
        BRM.setPower(power);
    }

    public void stop(){
        BLM.setPower(0);
        BRM.setPower(0);
        FLM.setPower(0);
        FRM.setPower(0);
    }

    public void turn(double power){
        BLM.setPower(power);
        BRM.setPower(-power);
        FLM.setPower(power);
        FRM.setPower(-power);
    }

    public void go2Point(int x, int y){
        while(getX() < x){
            sideways(1);
        }
        while(getX() > x){
            sideways(-.1);
        }
        stop();

        while(getY() < y){
            forward(1);
        }
        while(getY() > y){
            forward(-.1);
        }
        stop();
    }

    public double getAngle(){
        //set current angles to where the robot is facing
        Orientation currAngles=Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //angle change is the difference between where the robot is facing currently and where it used to face
        double angleChange=currAngles.firstAngle-prevAngles.firstAngle;
        //turning more than 180 degrees in either direction is the equivalent of turning
        //360 - the turn in the other direction
        if (angleChange<-180)
            angleChange+=360;
        else if (angleChange>180)
            angleChange-=360;

        //add the turn amount to the global robot orientation
        globalAngle+=angleChange;

        //set previous orientation to current orientation
        prevAngles=currAngles;
        //get angle getAngle()=globalAngle
        return globalAngle;
    }

    public void resetAngle(){
        //stores where the robot is facing as where the robot was previously facing
        prevAngles=Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // set where the robot is facing as 0
        globalAngle=0;
    }

    public double getAngleOdo(){
        return 2*Math.abs(getRight() - getLeft())/WHEEL_DIAMETER;
    }

    public double getAngleOdoDegrees(){
        return Math.toDegrees(getAngleOdo());
    }

    public void go2Point2(int xTarget, int yTarget){
        double dX = xTarget - getX();
        double dY = yTarget - getY();
        double theta = Math.atan2(dY, dX);
        double rot = Math.cos(getAngleOdo());
        double power1 = Math.tan(Math.tan(theta - Math.PI/4));
        double power2 = Math.tan(Math.tan(theta + Math.PI/4));

        while(getX() != xTarget && getY() != yTarget){
            if(theta >= 0 && theta < Math.PI/2){
                FRM.setPower(rot);
                FLM.setPower(power1 *  rot);
                BLM.setPower(rot);
                BRM.setPower(power1 *  rot);
            }
            else if(theta >= Math.PI/2 && theta < Math.PI){
                FRM.setPower(-power2 * rot);
                FLM.setPower(rot);
                BLM.setPower(-power2 * rot);
                BRM.setPower(rot);
            }
            else if(theta >= Math.PI && theta < 3*Math.PI/2){
                FRM.setPower(-rot);
                FLM.setPower(-power1 * rot);
                BLM.setPower(-rot);
                BRM.setPower(-power1 * rot);
            }
            else if(theta >= 3*Math.PI/2 && theta < 2*Math.PI){
                FRM.setPower(power2 *  rot);
                FLM.setPower(-rot);
                BLM.setPower(power2 *  rot);
                BRM.setPower(-rot);
            }
            else{
                break;
            }
        }
    }
}

