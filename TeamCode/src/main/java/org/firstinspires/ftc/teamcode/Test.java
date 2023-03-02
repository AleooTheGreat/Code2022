package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.MiniCookies;


@TeleOp(name="Test Servo", group="Linear Opmode")

public class Test extends LinearOpMode {

    MiniCookies mini ;

    boolean ok1 = false;
    boolean ok2 = false;
    boolean ok3 = false;
    boolean ok4 = false;

    @Override
    public void runOpMode() {

        mini = new MiniCookies(hardwareMap);


        mini.arm1.setPosition(0.34);
        mini.arm2.setPosition(0.34);

        waitForStart();

        //mini.arm(0.31);

        while (opModeIsActive()) {

       if(gamepad2.dpad_up && !ok1){
         mini.arm1.setPosition(mini.arm1.getPosition() + 0.05);
         ok1 = true;
       }
      if(gamepad2.dpad_down && !ok2){
          mini.arm1.setPosition(mini.arm1.getPosition() - 0.05);
          ok2 = true;
       }
      if(gamepad2.triangle && !ok3){
          mini.arm2.setPosition(mini.arm2.getPosition() + 0.05);
          ok3 = true;
       }
       if(gamepad2.cross && !ok4){
           mini.arm2.setPosition(mini.arm2.getPosition() - 0.05);
           ok4 = true;
       }

            if (!gamepad2.dpad_up) {
                ok1 = false;
            }

            if(!gamepad2.dpad_down){
                ok2 = false;
            }
            if(!gamepad2.triangle){
                ok3 = false;
            }
            if(!gamepad2.cross){
                ok4 = false;
            }

            telemetry.addData("sv1",mini.arm1.getPosition());
            telemetry.addData("sv2",mini.arm2.getPosition());
            telemetry.update();
        }
    }
}
