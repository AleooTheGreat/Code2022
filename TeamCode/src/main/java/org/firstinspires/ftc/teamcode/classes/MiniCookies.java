package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MiniCookies {

    public ServoImplEx sl = null;
    public ServoImplEx sr = null;
    public Servo lilpimp = null;

    public Servo odosp = null;

    private ElapsedTime runtime = new ElapsedTime();
    public double svpos = 0;

    private HardwareMap hwMap;

    public void init(HardwareMap hardwareMap){

        hwMap = hardwareMap;

        sl = hardwareMap.get(ServoImplEx.class, "sl");
        sr = hardwareMap.get(ServoImplEx.class, "sr");
        lilpimp = hardwareMap.get(Servo.class, "scr");
        odosp = hardwareMap.get(Servo.class, "odosp");

        sl.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        lilpimp.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);

        lilpimp.setPosition(0.2);
        sr.setPosition(svpos);
        sl.setPosition(svpos);
        odosp.setPosition(1);
    }

    public void open(){

        lilpimp.setPosition(0.15);
    }

    public void close(){

        lilpimp.setPosition(0);

    }

    public void openup(){
        lilpimp.setPosition(0.08);
    }
    public void up(){

        update_servo(0.635);
    }

    public void nohitup(){

        update_servo(0.56);
    }

    public void miniup(){

        update_servo(0.05);
    }
    public void down(){
        if(svpos == 0){

            sl.setPosition(0.30);
            sr.setPosition(0.30);

            runtime.reset();
            runtime.startTime();
            while(runtime.seconds() < 0.2){

            }
                sl.setPwmDisable();
                sr.setPwmDisable();

        }
    }

    public void update_servo(double svpos){

        sl.setPosition(svpos);
        sr.setPosition(svpos);
    }


}
