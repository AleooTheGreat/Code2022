package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Cookie {

    public DcMotor lf = null;
    public DcMotor lb = null;
    public DcMotor rf = null;
    public DcMotor rb = null;

    private HardwareMap hwMap;

    public void init(HardwareMap hardwareMap){
        hwMap = hardwareMap;

        lf = hardwareMap.dcMotor.get("lf");
        lb = hardwareMap.dcMotor.get("lb");
        rf = hardwareMap.dcMotor.get("rf");
        rb = hardwareMap.dcMotor.get("rb");

        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setPower(0);
        rb.setPower(0);
        rf.setPower(0);
        lb.setPower(0);


    }
}
