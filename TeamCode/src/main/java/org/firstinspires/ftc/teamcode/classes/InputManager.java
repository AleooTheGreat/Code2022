package org.firstinspires.ftc.teamcode.classes;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Function;
import java.util.function.Predicate;

public class InputManager {

    private Map<String, Button> buttonMap = new HashMap<String, Button>();
    private Map<String, Analog> analogMap = new HashMap<String, Analog>();
    public Gamepad _GAMEPAD1, _GAMEPAD2;

    public InputManager(Gamepad gp1, Gamepad gp2){
        _GAMEPAD1 = gp1;
        _GAMEPAD2 = gp2;
    }

    public void AddButton(String name, Gamepad gp, Predicate<Gamepad> function){
        buttonMap.put(name, new Button(gp, function));
    }
    public Button GetButton(String name){
        return buttonMap.get(name);
    }
    @RequiresApi(api = Build.VERSION_CODES.N)
    public void Update(){
        buttonMap.forEach((name, button) -> button.Handle());
        analogMap.forEach((name, analog) -> analog.Handle());
    }

    public void AddAnalog(String name, Gamepad gp, Function<Gamepad, Float> f, float dz){
        analogMap.put(name, new Analog(gp, f, dz));
    }
    public float GetAnalog(String name) {
        return analogMap.get(name).Value();
    }
    public boolean Tapped(String name){
        return GetButton(name).Tapped();
    }
}
