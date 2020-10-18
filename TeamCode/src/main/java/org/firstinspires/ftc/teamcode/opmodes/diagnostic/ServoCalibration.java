package org.firstinspires.ftc.teamcode.opmodes.diagnostic;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.libraries.hardware.BotHardware;

/**
 *  Made by Scott 10/18/20
 */
//TODO: Fix servo outOfIndex error
//TODO: Fix telemetry outputs

@TeleOp(name="Servo Calibrate")
public class ServoCalibration extends OpMode {
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
    boolean lastFaceButton = false;
    int incIndex = 0;

    int servoIndex = 0;
    Servo curServo;
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
        if(!lastFaceButton){
            if(gamepad1.b) servoIndex = Range.clip(servoIndex - 1, 0, BotHardware.ServoE.values().length - 1);
            else if(gamepad1.a) servoIndex = Range.clip(servoIndex + 1, 0, BotHardware.ServoE.values().length + 1);
        }
        curServo = BotHardware.ServoE.values()[servoIndex].servo;
        lastFaceButton = gamepad1.a || gamepad1.b;

        //telemetry
        telemetry.addData("Cur Servo", curServo);
        telemetry.addData("Servo Pos", curServo.getPosition());
        telemetry.addData("Increment", Incs.values()[incIndex].inc);
        telemetry.addData("Servo Index", servoIndex);

        if(!lastDPad){
            if(gamepad1.dpad_up)
                curServo.setPosition(Range.clip(curServo.getPosition() + Incs.values()[incIndex].inc, -1, 1));
            else if (gamepad1.dpad_down)
                curServo.setPosition(Range.clip(curServo.getPosition() - Incs.values()[incIndex].inc, -1, 1));

            else if (gamepad1.dpad_left) incIndex = Range.clip(incIndex - 1, 0, Incs.values().length - 1);
            else if (gamepad1.dpad_right) incIndex = Range.clip(incIndex + 1, 0, Incs.values().length - 1);
        }
        lastDPad = gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down;
    }
}
