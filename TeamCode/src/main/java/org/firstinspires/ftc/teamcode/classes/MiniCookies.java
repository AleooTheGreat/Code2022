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

    public ServoImplEx arm1 = null;
    public ServoImplEx arm2 = null;

    private ElapsedTime runtime = new ElapsedTime();

    public MiniCookies(HardwareMap hardwareMap){


        base = hardwareMap.get(DcMotorEx.class, "base");

        posa = hardwareMap.get(ServoImplEx.class, "posa");
        claw = hardwareMap.get(ServoImplEx.class, "claw");

        upl = hardwareMap.get(ServoImplEx.class, "upl");
        upr = hardwareMap.get(ServoImplEx.class, "upr");

        pick = hardwareMap.get(ServoImplEx.class, "pick");

        arm1 = hardwareMap.get(ServoImplEx.class, "arm1");
        arm2 = hardwareMap.get(ServoImplEx.class, "arm2");


        base.setDirection(DcMotorEx.Direction.REVERSE);
        upr.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);
        posa.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);
        arm1.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);
        pick.setDirection(com.qualcomm.robotcore.hardware.ServoImplEx.Direction.REVERSE);

        base.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        base.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        base.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pick.setPosition(0);

        poweroff();

        base.setTargetPosition(0);




    }

    public void arm(double position){
        arm1.setPosition(position);
        arm2.setPosition(position);
    }

    public void poweroff(){


        upl.setPwmDisable();
        upr.setPwmDisable();

        posa.setPwmDisable();
        claw.setPwmDisable();

        arm1.setPwmDisable();
        arm2.setPwmDisable();
    }

    public void startoff() {
        runtime.reset();
        while (runtime.seconds() < 1){
            posa.setPosition(0.54);

            claw.setPosition(0.47);
            pick.setPosition(0.489);

            arm1.setPosition(0.24);
            arm2.setPosition(0.24);
            if(runtime.seconds()<1.2) {
                upl.setPosition(0.14);
                upr.setPosition(0.14);
            }

        }
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

        claw.setPosition(0.66);

    }

    public void down(){

        open();

        posa.setPosition(0.5);

        arm1.setPosition(0.563);
        arm2.setPosition(0.563);

    }

    public void load(){
        close();

        runtime.reset();

        while(runtime.seconds()<1.25){
            if(runtime.seconds() > 0.25){
                arm1.setPosition(0.1772);
                arm2.setPosition(0.1772);
            }
            if(runtime.seconds()>0.45){
                posa.setPosition(0.243);

            }
            if(runtime.seconds()>0.75){
                base.setTargetPosition(0);
                base.setPower(1);
            }

            if(runtime.seconds() > 1.20){
                open();
            }
        }
       /* posa.setPosition(0.241);

        arm1.setPosition(0.307);
        arm2.setPosition(0.307);

        */
    }




    public void stack5(){
        open();
        posa.setPosition(0.57);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }



        arm1.setPosition(0.488);
        arm2.setPosition(0.488);




    }

    public void stack4(){

        open();
        posa.setPosition(0.5683);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }


        arm1.setPosition(0.502);
        arm2.setPosition(0.502);

    }

    public void stack3(){

        open();
        posa.setPosition(0.5294);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }

        arm1.setPosition(0.5183);
        arm2.setPosition(0.5183);



    }

    public void stack2(){

        open();
        posa.setPosition(0.527);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }
        arm1.setPosition(0.542);
        arm2.setPosition(0.542);


    }

    public void stack1(){

        open();

        posa.setPosition(0.5);

        runtime.reset();
        while(runtime.seconds()<0.3){
            if(runtime.seconds()>0.1){
                base.setTargetPosition(-544);
                base.setPower(1);
            }
        }

        arm1.setPosition(0.563);
        arm2.setPosition(0.563);

    }

}
