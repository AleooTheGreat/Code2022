package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MiniCookies {

    public ServoImplEx sl = null;
    public ServoImplEx sr = null;
    public ServoImplEx lilpimp = null;

    public Servo odosp = null;

    private ElapsedTime runtime = new ElapsedTime();
    public double svpos = 0;

    public MiniCookies(HardwareMap hardwareMap){

        sl = hardwareMap.get(ServoImplEx.class, "sl");
        sr = hardwareMap.get(ServoImplEx.class, "sr");
        lilpimp = hardwareMap.get(ServoImplEx.class, "scr");
        odosp = hardwareMap.get(Servo.class, "odosp");

        sl.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        lilpimp.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);

        lilpimp.setPosition(0.17);
        sr.setPosition(svpos);
        sl.setPosition(svpos);
        odosp.setPosition(1);
    }

    public void open(){

        lilpimp.setPosition(0.17);
    }

    public void close(){

        lilpimp.setPosition(0);

    }

    public void openup(){
        lilpimp.setPosition(0.06);
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

            sl.setPosition(0.0);
            sr.setPosition(0.0);

            runtime.reset();
            while(runtime.seconds() < 0.26){
            }
            sl.setPwmDisable();
            sr.setPwmDisable();

    }

    public void update_servo(double svpos){

        sl.setPosition(svpos);
        sr.setPosition(svpos);

        if(svpos == 0.635){
            runtime.reset();
            while(runtime.seconds() < 1){
                if(runtime.seconds() > 0.65){
                    sl.setPwmDisable();
                    sr.setPwmDisable();
                }
            }
        }
    }


}
