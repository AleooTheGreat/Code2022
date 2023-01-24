/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.classes.Cookie;
import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;


@TeleOp(name="JamalOp", group="Iterative Opmode")
@Config

public class JamalOP extends OpMode
{

    Cookie cookie = new Cookie();
    MiniCookies minicookies = new MiniCookies();
    GetCookies lift = new GetCookies();



    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime timer = new ElapsedTime();

    boolean ChangeSpeed = false;

    boolean LowJ = false, MidJ = false, HighJ = false;

    double speedM = 1f;




    ElapsedTime ServoRunTime = new ElapsedTime();

    //Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry tel = dashboard.getTelemetry();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        cookie.init(hardwareMap);
        lift.init(hardwareMap);
        minicookies.init(hardwareMap);
        minicookies.iohann.setPosition(0.8);
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Gamepad1

        if (gamepad1.cross && !ChangeSpeed) {
            speedM = 0.3f;
            ChangeSpeed = true;
        } else if (gamepad1.cross && ChangeSpeed) {
            speedM = 1f;
            ChangeSpeed = false;
        }

        if (gamepad1.dpad_up) {
            cookie.rb.setPower(1);
        }
        if (gamepad1.dpad_down) {
            cookie.rf.setPower(1);
        }
        if (gamepad1.dpad_right) {
            cookie.lf.setPower(1);
        }
        if (gamepad1.dpad_left) {
            cookie.lb.setPower(1);
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.3;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double lfp = (y + x + rx) / denominator;
        double lbp = (y - x + rx) / denominator;
        double rfp = (y - x - rx) / denominator;
        double rbp = (y + x - rx) / denominator;

        cookie.lf.setPower(lfp * speedM);
        cookie.lb.setPower(lbp * speedM);
        cookie.rf.setPower(rfp * speedM);
        cookie.rb.setPower(rbp * speedM);

        //Gamepad2

        if(gamepad2.dpad_up && !HighJ) {
            lift.up(2);
            HighJ = true;

        } else if(gamepad2.dpad_up && HighJ) {
             lift.down();
             HighJ = false;

        }

        if(gamepad2.dpad_left && !MidJ){
            lift.up(1);
            MidJ = true;

        } else if(gamepad2.dpad_left){
            lift.down();
            MidJ = false;
        }


        if(gamepad2.dpad_down && !LowJ){

            minicookies.up();


            LowJ = true;

        }else if(gamepad2.dpad_down && LowJ){

            minicookies.down();


            LowJ = false;
        }


        if(gamepad2.square){
            minicookies.sr.setPosition(0.66);
            minicookies.sl.setPosition(0.66);
        }
        if(gamepad2.circle){
            minicookies.sr.setPosition(0);
            minicookies.sl.setPosition(0);
        }

        if(gamepad2.right_bumper){
           minicookies.lilpimp.setPosition(0.1);
           if(HighJ || MidJ || LowJ) {
               minicookies.miniup();
           }
        }
        if(gamepad2.left_bumper){
            minicookies.lilpimp.setPosition(0);
            if(!HighJ||!MidJ||!LowJ){
                minicookies.miniup();
            }
            minicookies.minidown();
        }

        telemetry.addData("position", GetCookies.setpoint);
        telemetry.addData("gl_pos", lift.gl.getCurrentPosition());
        telemetry.addData("gr_pos", lift.gr.getCurrentPosition());
        telemetry.update();

        lift.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}


