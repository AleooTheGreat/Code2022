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

import  com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.GetCookies;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="Doamne Miluieste", group="Linear Opmode")

public class OfficialOP extends LinearOpMode {

    SampleMecanumDrive cookie;
    MiniCookies minicookies;
    GetCookies lift;


    int last_position = 0;

    int position = 0;
    double speedM = 0.85f;
    boolean up = false;
    boolean will = true;
    boolean ok_speed = false;

    boolean highj = false;
    boolean midj = false;

    @Override
    public void runOpMode() {

        cookie = new SampleMecanumDrive(hardwareMap);
        minicookies = new MiniCookies(hardwareMap);
        lift = new GetCookies(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        final Thread high = new Thread() {
            public void run() {
                lift.up(2);
                minicookies.pick.setPosition(0.660);
                OfficialOP.this.sleep(750);
                minicookies.put();
            }
        };

        final Thread down_mid = new Thread() {
            public void run() {
                minicookies.take();
                OfficialOP.this.sleep(250);
                lift.down();
                OfficialOP.this.sleep(250);
                minicookies.pick.setPosition(0.56);
            }
        };

        final Thread down_high = new Thread() {
            public void run() {
                minicookies.take();
                OfficialOP.this.sleep(250);
                lift.down();
                OfficialOP.this.sleep(350);
                minicookies.pick.setPosition(0.56);
            }
        };

        final Thread mid = new Thread() {
            public void run() {
                lift.up(1);
                minicookies.pick.setPosition(0.71);
                OfficialOP.this.sleep(250);
                minicookies.put();
            }
        };

        minicookies.startoff();

        waitForStart();

        //after start;

        while (opModeIsActive()) {

            if (gamepad1.cross && !ok_speed) {
                ok_speed = true;
                if (speedM == 0.85f) {
                    speedM = 2f;
                } else {
                    speedM = 0.9f;
                }
            }

            if (!gamepad1.cross) {
                ok_speed = false;
            }

            double y = -gamepad1.left_stick_y * 0.7;
            double x = gamepad1.left_stick_x * 1.3;
            double rx = gamepad1.right_stick_x;

            cookie.setWeightedDrivePower(new Pose2d(y * speedM,
                    -x * 0.3,
                    -rx * speedM));

            //༼ つ ◕_◕ ༽つ🍪
            //Gamepad2


            if (gamepad2.left_bumper) {
                minicookies.take();
            }
            if (gamepad2.right_bumper) {
                minicookies.put();
            }

            if (gamepad2.dpad_right) {
                position = last_position;
                minicookies.down();
                speedM = 0.9f;
            }

            if (gamepad2.cross) {
                last_position = minicookies.base.getCurrentPosition();
                position = 0;
                minicookies.load();
                speedM = 0.9f;
            }


            //Levels
            if (gamepad2.dpad_down) {
                if(midj) {
                    midj = false;
                    down_mid.start();
                }else if(highj){
                    highj = false;
                    down_high.start();
                }
                speedM = 0.65f;
            }

            if (gamepad2.dpad_left) {
                midj = true;
                mid.start();
                up = true;
            }

            if (gamepad2.dpad_up) {
                highj = true;
                high.start();
                up = true;
            }

            if (gamepad2.circle) {
                minicookies.close();
            }

            if(gamepad2.square){
                minicookies.open();
            }

            if(gamepad2.triangle){
                position = 0;
            }

            position += (int)(gamepad2.left_stick_x * 10);
            minicookies.base.setTargetPosition(position);
            minicookies.base.setPower(1);

            minicookies.pick.setPosition(minicookies.pick.getPosition() + gamepad2.right_stick_x/100);
            minicookies.arm(minicookies.arm1.getPosition() + gamepad2.left_stick_y/100);
            minicookies.posa.setPosition(minicookies.posa.getPosition() + gamepad2.right_stick_y/100);

            telemetry.addData("rotatie", minicookies.base.getCurrentPosition());
             telemetry.addData("arm_pos", minicookies.arm1.getPosition());
            telemetry.addData("arm_pos2", minicookies.arm2.getPosition());
            telemetry.addData("pozarm", minicookies.posa.getPosition());
            telemetry.addData("picki",minicookies.pick.getPosition());
            telemetry.addData("lastpos",last_position);
            telemetry.addData("pos",position);
            telemetry.addData("will", will);
            telemetry.addData("gl_st",lift.gl.getCurrentPosition());
            telemetry.addData("gl_rg",lift.gr.getCurrentPosition());
            telemetry.update();


            lift.update();


        }
    }
}
