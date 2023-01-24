package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class GetCookies {
    public DcMotorEx gl = null;
    public DcMotorEx gr = null;

    private PIDController controllerl;
    private PIDController controllerr;

    public static double kP = 0.01;
    public static double kI = 0.0001;
    public static double kD = 0.000004;
    public static double kF = 0.006;

    public static double setpoint = 0;

    double ticks_in_degrees = 145.1 / 360;

    private HardwareMap hwMap;

    MiniCookies minicookies = new MiniCookies();

    public void init(HardwareMap hardwareMap) {

        hwMap = hardwareMap;

        gl = hardwareMap.get(DcMotorEx.class, "lg");
        gr = hardwareMap.get(DcMotorEx.class, "rg");

        gl.setDirection(DcMotorSimple.Direction.REVERSE);

        gl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        gr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        gl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        gr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        gl.setPower(0);
        gr.setPower(0);

        controllerl = new PIDController(kP,kI,kD);
        controllerr = new PIDController(kP,kI,kD);
    }

    public void update(){

        if(setpoint != 0) {

            controllerl.setPID(kP, kI, kD);
            int glPos = gl.getCurrentPosition();
            double pidl = controllerl.calculate(glPos, setpoint);
            double ffl = Math.cos(Math.toRadians(setpoint / ticks_in_degrees)) * kF;

            double powerl = pidl + ffl;

            gl.setPower(powerl);

            controllerr.setPID(kP, kI, kD);
            int grPos = gr.getCurrentPosition();
            double pidr = controllerr.calculate(grPos, setpoint);
            double ffr = Math.cos(Math.toRadians(setpoint / ticks_in_degrees)) * kF;

            double powerr = pidr + ffr;

            gr.setPower(powerr);
        }else if(gl.getCurrentPosition() == 0 && gr.getCurrentPosition() == 0){
            gl.setPower(0);
            gr.setPower(0);
        }
    }


    public void down(){

        setpoint = 0;
        if(gl.getCurrentPosition() < 40 && gr.getCurrentPosition() < 40){
            minicookies.down();
        }

    }

    public void up(int level){
        if(level == 1){
            setpoint = 500;
        }else if(level == 2){
            setpoint = 940;
        }else{
            setpoint = 0;
        }
    }

}
