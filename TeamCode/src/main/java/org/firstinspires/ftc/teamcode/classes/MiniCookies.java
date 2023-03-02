package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class MiniCookies {


    public ServoImplEx upl = null;
    public ServoImplEx upr = null;
    public ServoImplEx posa = null;
    public ServoImplEx claw= null;
    public ServoImplEx pick = null;

    public DcMotorEx base = null;

    private ElapsedTime runtime = new ElapsedTime();

    public MiniCookies(HardwareMap hardwareMap){


        base = hardwareMap.get(DcMotorEx.class, "base");

        posa = hardwareMap.get(ServoImplEx.class, "posa");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        upl = hardwareMap.get(ServoImplEx.class, "upl");
        upr = hardwareMap.get(ServoImplEx.class, "upr");

        pick = hardwareMap.get(ServoImplEx.class, "pick");


        base.setDirection(DcMotorEx.Direction.REVERSE);
        upr.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);
        posa.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);
        pick.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);

        base.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        base.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        base.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        poweroff();

        base.setTargetPosition(0);




    }

    public void poweroff(){


        upl.setPwmDisable();
        upr.setPwmDisable();

        posa.setPwmDisable();
        claw.setPwmDisable();
    }

    public void startoff(){

        upl.setPosition(0.14);
        upr.setPosition(0.14);

        posa.setPosition(0.5);

        claw.setPosition(0.47);
        pick.setPosition(0.1);
    }

    public void take(){
        upl.setPosition(0.14);
        upr.setPosition(0.14);
    }

    public void put(){
        upl.setPosition(0.75);
        upr.setPosition(0.75);

    }

    public void open(){

        claw.setPosition(0.47);
    }

    public void close(){

        claw.setPosition(0.75);

    }


    public void down(){
        posa.setPosition(0.5);
        open();
    }


    public void load() {

        close();

        runtime.reset();

        while (runtime.seconds() < 1) {

            if(runtime.seconds()> 0.15) {
            }


            if (runtime.seconds() > 0.3) {
                base.setTargetPosition(0);
                base.setPower(1);
            }

            if(runtime.seconds() > 0.4){
                posa.setPosition(0.207);
            }

            if (runtime.seconds() > 0.95) {

                claw.setPosition(0.47);

            }

        }
    }

    public void stack5(){

        open();
        posa.setPosition(0.565);
        base.setTargetPosition(-544);
        base.setPower(1);


    }

    public void stack4(){

        open();
        posa.setPosition(0.5594);
        base.setTargetPosition(-541);
        base.setPower(1);


    }

    public void stack3(){

        open();
        posa.setPosition(0.5011);
        base.setTargetPosition(-544);
        base.setPower(1);


    }

    public void stack2(){

        open();
        posa.setPosition(0.5);
        base.setTargetPosition(-550);
        base.setPower(1);

    }

    public void stack1(){

        open();
        posa.setPosition(0.5);
        base.setTargetPosition(-550);
        base.setPower(1);


    }

    public void waitforload(){
        close();

        runtime.reset();

        while (runtime.seconds() < 1) {

            if (runtime.seconds() > 0.2) {



            }

            if(runtime.seconds() > 0.4){

                posa.setPosition(0.235);


            }
        }
    }

    public void afterload(){

        runtime.reset();

        while (runtime.seconds() < 1) {

            if (runtime.seconds() > 0.2) {


            }

            if (runtime.seconds() > 0.55) {

                claw.setPosition(0.47);

            }

        }
    }

}
