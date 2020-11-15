package org.firstinspires.ftc.teamcode.opmodes;

//import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.hardware.IanHardware;
import org.firstinspires.ftc.teamcode.libraries.interfaces.ControllerLib;

@TeleOp(name="Ian learning code lol")
public class IanLeraning extends OpMode {
    private static final float motorPow = .5f;

    protected IanHardware rat = new IanHardware(this);
    private ControllerLib g1;

    @Override
    public void init(){
        rat.init();
        g1 = new ControllerLib(gamepad1);
    }

    @Override
    public void start(){
        gamepad1.setJoystickDeadzone(0.05f);
        rat.start(); //yes absolutely necessary

    }

    @Override
    public void loop(){
        g1.update();

        float ly = g1.left_stick_y;
        float ry = g1.right_stick_y;

        double lPow = ly;
        double rPow = ry;
        lPow = Range.clip(lPow, -1, 1);
        rPow = Range.clip(rPow, -1, 1);
        rat.setAllDrive(ry, ry, ly, ly);
    }

    @Override
    public void stop() {rat.stopAll();}
}