package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class JoystickRumble extends OpMode {
    double endGameStarttime;

    boolean endgamestatus;
    @Override
    public void init() {
    }

    @Override
    public void start(){
        endGameStarttime = getRuntime() + 90;
    }

    @Override
    public void loop() {
        if(getRuntime() >= endGameStarttime && !endgamestatus){
            gamepad1.rumbleBlips(3);
            endgamestatus = true;
        }

    }
}