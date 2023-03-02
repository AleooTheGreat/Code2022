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

    public static double maxPIDPower = 0.5;
    public static double kP = 0.003;
    public static double kI = 0;
    public static double kD = 0.0004;
    public static double kF = 0.004;

    public static double setpoint = 0;

    double ticks_in_degrees = 145.1 / 360;

    public Arm(HardwareMap hardwareMap) {

        setpoint = 0;

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
            double pid = Math.min(controller.calculate(armPos, setpoint), maxPIDPower);
            double ffl = Math.cos(Math.toRadians(setpoint / ticks_in_degrees)) * kF;

            double power = pid + ffl;

            arm.setPower(power);


    }


}
