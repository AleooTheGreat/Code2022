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

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.classes.Cookie;
import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name="JamalOp", group="Iterative Opmode")
@Config

public class JamalOP extends OpMode
{

    SampleMecanumDrive cookie;
    MiniCookies minicookies;
    GetCookies lift;

    ElapsedTime runtime = new ElapsedTime();


    boolean auto = false;
    boolean open = false;
    double brat = 0;

    boolean noupstate = false, sus = false;
    boolean fail = false;

    double speedM = 0.85f;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        cookie = new SampleMecanumDrive(hardwareMap);
        minicookies = new MiniCookies(hardwareMap);
        lift = new GetCookies(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        minicookies.odosp.setPosition(0.5);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {


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
            if(speedM == 0.85f)
            {
                speedM = 2f;
            }
            else
            {
                speedM = 0.85f;
            }
        }

        if(!gamepad1.cross)
        {
            ok_speed = false;
        }

        double y = -gamepad1.left_stick_y * 0.7;
        double x = gamepad1.left_stick_x * 1.3;
        double rx = gamepad1.right_stick_x;

        cookie.setWeightedDrivePower(new Pose2d(y * speedM,
                -x * 0.3,
                -rx * speedM));

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
       /*if(gamepad2.dpad_up) {

            lift.up(2);
            minicookies.up();

            sus = true;
            noupstate = true;



           minicookies.update_servo(0.19);
        }
        */



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
            noupstate = false;

        }


        //cleste




        if(gamepad2.square){
            minicookies.down();
        }

        if(gamepad2.right_bumper){

            if(open||sus){
                minicookies.openup();
                open = false;
            }else {
                minicookies.open();
            }

          if(sus){
              while(runtime.seconds() < 0.250){

              }
              minicookies.nohitup();
          }

            fail = false;

        }


        if(gamepad2.left_bumper && !auto){

            minicookies.close();

            if(!noupstate){
                runtime.reset();
                while(runtime.seconds() < 0.250){

                }
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
        telemetry.addData("val",brat);
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


