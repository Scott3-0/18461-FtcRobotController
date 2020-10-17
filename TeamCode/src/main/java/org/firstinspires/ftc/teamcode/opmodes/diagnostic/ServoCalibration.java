package org.firstinspires.ftc.teamcode.opmodes.diagnostic;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.libraries.hardware.BotHardware;

@TeleOp(name="Servo Calibrate")
public class ServoCalibration extends OpMode {
    int i = 0;
    private enum Incs {
        SMALL(0.01),
        MEDIUM(0.05),
        EHH(0.1),
        HOLYMOTHER(0.5);

        public double inc;
        Incs(double inc) {
            this.inc = inc;
        }
    }

    BotHardware bot = new BotHardware(this);
    boolean lastDPad = false;
    int incIndex = 0;

    @Override
    public void init() {

        telemetry.addData("Status", "Initialized");

        //currentServo = robot.leftServo;

        // hardware maps
        bot.init();
    }

    @Override
    public void start() {}

    @Override
    public void loop() {

        i = Seti(i);
        telemetry.addData("Servo", bot.GetServo(i).getPosition());
        telemetry.addData("Increment", Incs.values()[incIndex].inc);
        telemetry.addData("Index", i);
        telemetry.addData("Servo Name", bot.GetServo(i).getDeviceName());

        if(!lastDPad) {
            if (gamepad1.dpad_up)
                bot.GetServo(i).setPosition(Range.clip(bot.GetServo(i).getPosition() + Incs.values()[incIndex].inc, -1, 1));
            else if (gamepad1.dpad_down)
                bot.GetServo(i).setPosition(Range.clip(bot.GetServo(i).getPosition() - Incs.values()[incIndex].inc, -1, 1));
            else if (gamepad1.dpad_left) incIndex = Range.clip(incIndex - 1, 0, Incs.values().length - 1);
            else if (gamepad1.dpad_right) incIndex = Range.clip(incIndex + 1, 0, Incs.values().length - 1);
        }

        lastDPad = gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down;
    }
    int Seti(int in){
        if(gamepad1.a && in < BotHardware.ServoE.values().length){
            return in++;
        }
        else if(gamepad1.b && in > 0){
            return in--;
        }
        else{
            return in;
        }
    }
}
