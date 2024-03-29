package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Arm {
    public DcMotorEx arm = null;

    private PIDController controller;

    public static double maxPIDPower = 1;
    public static double kP = 0.00174;
    public static double kI = 0;

    public static double kD = 0.00035;
    public static double kF = 0.003;
    public static int offset = 460;

    public static double setpoint_arm = 0;

    public double  ticks_in_degrees = 1153.6193 / 360;

    boolean ok = false;

    public Arm(HardwareMap hardwareMap) {

        setpoint_arm = 0;

        arm = hardwareMap.get(DcMotorEx.class, "arm");

        arm.setDirection(DcMotorEx.Direction.REVERSE);

        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        arm.setPower(0);


        controller = new PIDController(kP,kI,kD);
    }

    public void update(){


            controller.setPID(kP, kI, kD);
            int armPos = arm.getCurrentPosition();
            double pid = Math.min(controller.calculate(armPos, setpoint_arm), maxPIDPower);
             double ffl = Math.cos(Math.toRadians((setpoint_arm - offset) / ticks_in_degrees)) * -kF;

            double power = pid + ffl;

            arm.setPower(power);


    }

    public void modify(){
        offset = 0;
        ok = true;
    }

    public void up_arm(){

        if(ok){
            setpoint_arm = -345;
        }else{
            setpoint_arm = 115;
        }
    }

    public void down_arm(){
        if(ok){
            setpoint_arm = 15;
        }else {
            setpoint_arm = 475;
        }
    }

    public void up_arm_to_pos(int postion){
        setpoint_arm = postion;
    }


}
