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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.Arm;
import org.firstinspires.ftc.teamcode.classes.MiniCookies;


@TeleOp(name="Arm", group="Iterative Opmode")
@Config
public class Test_Arm extends OpMode
{

   // Arm arm ;
    MiniCookies mini;

   // public static double position = 0.285;
    public static double otherposition = 0.47;
    @Override
    public void init() {
       // arm = new Arm(hardwareMap);
        mini = new MiniCookies(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mini.pick.setPosition(0.5);
        mini.arm1.setPosition(0.23);
        mini.arm2.setPosition(0.23);
    }

    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }


    @Override
    public void loop() {

        if(gamepad2.circle){
            mini.open();
        }

        if(gamepad2.square){
            mini.close();
        }

        if(gamepad2.cross){
            mini.arm1.setPosition(0.23);
            mini.arm2.setPosition(0.23);
        }

        if(gamepad2.triangle){
            mini.arm1.setPosition(0.65);
            mini.arm2.setPosition(0.65);
        }

        if(gamepad2.right_bumper){
            mini.upl.setPosition(0.74);
            mini.upr.setPosition(0.74);
        }

        if(gamepad2.left_bumper){
            mini.upl.setPosition(0.14);
            mini.upr.setPosition(0.14);
        }

        mini.arm1.setPosition(mini.arm1.getPosition() + gamepad2.left_stick_y/100);
        mini.arm2.setPosition(mini.arm2.getPosition() + gamepad2.left_stick_y/100);
        mini.pick.setPosition(mini.pick.getPosition() + gamepad2.right_stick_y/100);

   //     mini.pick.setPosition(position);
    //    mini.claw.setPosition(otherposition);

        telemetry.addData("arm1",mini.arm1.getPosition()) ;
        telemetry.addData("arm2", mini.arm2.getPosition());
        telemetry.addData("posarm",mini.posa.getPosition());
        telemetry.update();
    }


    @Override
    public void stop() {
    }

}
