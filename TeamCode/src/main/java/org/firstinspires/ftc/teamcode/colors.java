package org.firstinspires.ftc.teamcode;

import org.opencv.core.Scalar;

public class colors {

    Scalar lower = new Scalar(0, 0, 0);

    public colors(){

    }

    public enum colorChoices{
        YELLOW,
        PINK,
        GREEN,
        BLUE;
    }

    public Scalar lower(int color){
        switch(color){
            //yellow, pink, green, blue
            case 0:
                return new Scalar(80, 40, 200);
            case 1:
                return new Scalar(120 , 75, 175);
            case 2:
                return new Scalar(45, 60, 60);
            case 3:
                return new Scalar(0, 123, 51);
            default:
                return new Scalar(0, 0, 0);
        }
    }
//red [80, 99] [120, 150], [45, 70], [0,27]
    public Scalar upper(int color){
        switch(color){
            //yellow, red, green, blue
            case 0:
                return new Scalar(99, 255, 255);
            case 1:
                return new Scalar(150 , 255, 255);
            case 2:
                return new Scalar(70, 255, 255);
            case 3:
                return new Scalar(27, 255, 255);
            default:
                return new Scalar(255, 255, 255);
        }
    }
}
