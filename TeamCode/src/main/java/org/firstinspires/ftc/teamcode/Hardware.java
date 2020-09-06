package org.firstinspires.ftc.teamcode;

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

    public DcMotor encoderX;
    public DcMotor encoderRight;
    public DcMotor encoderLeft;

    public Blinker expansion_Hub_2;

    public ElapsedTime RunTime = new ElapsedTime();

    BNO055IMU Imu;
    public Orientation prevAngles = new Orientation();
    public Gyroscope imu;

    public double WHEEL_DIAMETER = 1.37795;
    public double TICKS_PER_REV = 1;
    public double ROT_WHEEL_DISTANCE = 12;

    public int prevX = 0;
    public int prevRight = 0;
    public int prevLeft = 0;

    int x;
    int y;
    double rotY = 0;
    double rotX = 0;

    int startY = 0;
    int startX = 0;

    public Hardware() {

    }

    public void init(HardwareMap hardwareMap) {
        //Getting Motor Name From Phone
        BLM = hardwareMap.dcMotor.get("BLM");
        BRM = hardwareMap.dcMotor.get("BRM");
        FLM = hardwareMap.dcMotor.get("FLM");
        FRM = hardwareMap.dcMotor.get("FRM");

        encoderX = hardwareMap.dcMotor.get("encoderX");
        encoderRight = hardwareMap.dcMotor.get("encoderRight");
        encoderLeft = hardwareMap.dcMotor.get("encoderLeft");

        // showing which direction the robot moves
        FRM.setDirection(DcMotorSimple.Direction.REVERSE);
        BRM.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public double getRight(){
        return (encoderRight.getCurrentPosition() / TICKS_PER_REV * Math.PI * WHEEL_DIAMETER) + startY;
    }

    public double getLeft(){
        return (encoderLeft.getCurrentPosition() / TICKS_PER_REV * Math.PI * WHEEL_DIAMETER) + startY;
    }

    public double getX(){
        return (encoderX.getCurrentPosition() / TICKS_PER_REV * Math.PI * WHEEL_DIAMETER) + startX;
    }

    public double getY(){
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

    public double getAngleIMU(){
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

    public void resetAngleIMU(){
        //stores where the robot is facing as where the robot was previously facing
        prevAngles=Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // set where the robot is facing as 0
        globalAngle=0;
    }

    public double getAngleOdo(){
        return (getLeft() - getRight())/ROT_WHEEL_DISTANCE;
    }

    public int getAngleOdoDegrees(){
        return (int) Math.toDegrees(getAngleOdo());
    }

    public void go2Point(int xTarget, int yTarget){
        while(x != xTarget || y != yTarget){
            double dX = xTarget - x;
            double dY = yTarget - y;
            double theta = Math.atan2(dY, dX) + getAngleOdo();

            while(theta > Math.PI){
                theta -= 2*Math.PI;
            }
            while(theta <= -Math.PI){
                theta += 2*Math.PI;
            }
            updatePos();

            double power1 = Math.tan(theta - Math.PI/4);
            double power2 = Math.tan(theta + Math.PI/4);
            if(theta > -Math.PI && theta <= -Math.PI/2){
                FRM.setPower(-power1);
                FLM.setPower(-1);
                BLM.setPower(-power1);
                BRM.setPower(-1);
            }
            else if(theta > -Math.PI/2 && theta <= 0){
                FRM.setPower(-1);
                FLM.setPower(power2);
                BLM.setPower(-1);
                BRM.setPower(power2);
            }
            else if(theta > 0 && theta <= Math.PI/2){
                FRM.setPower(power1);
                FLM.setPower(1);
                BLM.setPower(power1);
                BRM.setPower(1);
            }
            else if(theta > Math.PI/2 && theta <= Math.PI){
                FRM.setPower(1);
                FLM.setPower(-power2);
                BLM.setPower(1);
                BRM.setPower(-power2);
            }
        }
        stop();
    }

    public void turnOdo(int angle){
        double inX = x;
        double inY = y;
        while(getAngleOdoDegrees() != angle){
            double angDiff = Math.abs(angle - getAngleOdoDegrees());
            double power;
            if(angDiff <= 45){
                power = (double) (angle - getAngleOdoDegrees()) / 45;
            }
            else{
                power = angle - getAngleOdoDegrees();
            }
            turn(power);
            updatePos();
        }
        stop();
        rotX += x - inX;
        rotY += y - inY;
    }

    public void turnIMU(int angle){
        double angDiff = angle - getAngleOdoDegrees();
        while(getAngleOdoDegrees() != angle){
            double power = (angle - getAngleOdoDegrees()) / angDiff;
            turn(power);
        }
    }

    public void initEncoders(int startX, int startY){
        this.startX = startX;
        this.startY = startY;
    }

    //update in angle causes change to x and y
    public void updatePos(){
        x = (int) (getX() * Math.cos(getAngleOdo()) + getY() * Math.sin(getAngleOdo()) - rotX);
        y = (int) (getX() * -Math.sin(getAngleOdo()) + getY() * Math.cos(getAngleOdo()) - rotY);
    }
}
