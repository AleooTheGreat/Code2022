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


import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.util.ElapsedTime;


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


    boolean auto = false;
    boolean open = false;
    double brat = 0;

    boolean noupstate = false, sus = false;
    boolean fail = false;

    double speedM = 0.8f;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {



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
        minicookies.odosp.setPosition(0.8);
    }


    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();

    }

    private boolean ok_speed = false;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        //Gamepad1
        if(gamepad1.cross && !ok_speed)
        {
            ok_speed = true;
            if(speedM == 0.8f)
            {
                speedM = 2f;
            }
            else
            {
                speedM = 0.8f;
            }
        }

        if(!gamepad1.cross)
        {
            ok_speed = false;
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

        double y = -gamepad1.left_stick_y * 0.7;
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


        if(gamepad2.triangle){
            auto = true;
        }

        if(gamepad2.dpad_right){
            auto = false;
        }

        if(auto){
            brat += (gamepad2.left_stick_y/50);
            minicookies.update_servo(-brat);
        }

        //Glisiere
       if(gamepad2.dpad_up) {

            lift.up(2);
            minicookies.up();

            sus = true;
            noupstate = true;

        }



        if(gamepad2.dpad_left){

            lift.up(1);
            minicookies.up();

            sus = true;
            noupstate = true;

        }


        if(gamepad2.circle){

            lift.down();
            
            sus = false;
            noupstate = false;

        }


        //brat carbon
        if(gamepad2.dpad_down){

            minicookies.up();
            open = true;
            noupstate = true;

        }
        if(gamepad2.cross){

            minicookies.close();
            minicookies.down();
            sleep(500);
            minicookies.open();
            noupstate = false;

        }


        //cleste




        if(gamepad2.square){
            minicookies.update_servo(0);
        }

        if(gamepad2.right_bumper){

            if(open||sus){
                minicookies.openup();
            }else {
                minicookies.open();
            }

          if(sus){
              sleep(250);
              minicookies.nohitup();
          }

            fail = false;

        }


        if(gamepad2.left_bumper && !auto){

            minicookies.close();

            if(!noupstate){

                sleep(250);

                minicookies.miniup();

                fail = true;

            }
        }else if(gamepad2.left_bumper && auto){
            minicookies.close();
        }



        //telemetry verificare
        telemetry.addData("value", auto);
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


