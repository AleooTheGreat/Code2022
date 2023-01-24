package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class MiniCookies {

    public Servo sl = null;
    public Servo sr = null;
    public Servo lilpimp = null;

    public Servo iohann = null;

    private HardwareMap hwMap;

    public void init(HardwareMap hardwareMap){

        hwMap = hardwareMap;

        sl = hardwareMap.get(Servo.class, "sl");
        sr = hardwareMap.get(Servo.class, "sr");
        lilpimp = hardwareMap.get(Servo.class, "scr");
        iohann = hardwareMap.get(Servo.class, "odosp");

        sr.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        lilpimp.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);

        lilpimp.setPosition(0);
        sr.setPosition(0);
        sl.setPosition(0);
        iohann.setPosition(1);
    }

    public void up(){
        sl.setPosition(0.66);
        sr.setPosition(0.66);
    }

    public void miniup(){
        sl.setPosition(0.15);
        sr.setPosition(0.15);
    }
    public void down(){
        sl.setPosition(0);
        sr.setPosition(0);
    }

    public void minidown(){
        sl.setPosition(0.5);
        sr.setPosition(0.5);
    }

}
