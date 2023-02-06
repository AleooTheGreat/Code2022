package org.firstinspires.ftc.teamcode.classes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Function;

public class Analog {
    private Function<Gamepad, Float> function;
    private Gamepad gamepad;
    private float deadZone = 0.1f;
    private float value = 0;

    public Analog(Gamepad gp, Function<Gamepad, Float> f, float dz){
        function = f;
        gamepad = gp;
        deadZone = dz;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void Handle(){
        value = function.apply(gamepad);
        if(Math.abs(value) < deadZone)
            value = 0;
    }

    public float Value(){
        return value;
    }
}
