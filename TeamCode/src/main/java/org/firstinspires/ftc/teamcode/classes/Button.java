package org.firstinspires.ftc.teamcode.classes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.function.Predicate;

public class Button {
    private Predicate<Gamepad> predicate;
    private boolean fired = false;
    private boolean held = false;
    private Gamepad gamepad;

    public Button(Gamepad gp, Predicate<Gamepad> function){
        predicate = function;
        gamepad = gp;
    }

    @RequiresApi(api = Build.VERSION_CODES.N)
    public void Handle(){
        if(predicate.test(gamepad)){
            if(!held)
                fired = true;
            else
                fired = false;
            held = true;
        } else {
            held = false;
            fired = false;
        }
    }

    public boolean Tapped(){
        return fired;
    }
    public boolean On(){
        return held;
    }
}
