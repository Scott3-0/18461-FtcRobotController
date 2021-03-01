 package org.firstinspires.ftc.teamcode.opmodes.diagnostic;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.SensorLib;
import org.firstinspires.ftc.teamcode.libraries.hardware.BotHardware;
import org.firstinspires.ftc.teamcode.libraries.interfaces.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode.libraries.interfaces.ControllerLib;

import java.text.DecimalFormat;

//check this shiz out https://pidexplained.com/how-to-tune-a-pid-controller/

@TeleOp(name ="PID Tuner")
public class PidTuner extends OpMode {
    // construct a PID controller for correcting heading errors
    double Kp = 0.05f;        // degree heading proportional term correction per degree of deviation
    double Ki = 0.11f;        // ... integrator term
    double Kd = 0f;         // ... derivative term
    double KiCutoff = 7.5f;   // maximum angle error for which we update integrator
    SensorLib.PID pid;
    private enum Incs {
        MEWHENSMALL(0.001),
        SMALL(0.01),
        MEDIUM(0.05),
        EHH(0.1),
        BIGGISH(0.5),
        BIG(1),
        BIGGER(5),
        BIGGEST(10),
        HOLYMOTHER(50),
        HOLYFATHER(100),
        HOLY(500),
        HOLIEST(1000);

        public double inc;
        Incs(double inc) {
            this.inc = inc;
        }
    }
    boolean lastDPad = false;
    boolean lastFaceButton = false;
    int incIndex = 0;

    int curSelectVar = 0;
    private static final float slowPow = 0.33f;
    private static final float fastPow = 1.0f;
    private boolean robotSlow = false; //init value
    private boolean lastA = false;

    private ControllerLib g1;
    private ControllerLib g2;

    protected BotHardware bot = new BotHardware(this);

    private BNO055IMUHeadingSensor localIMU = null;
    float initialHeading = 0.0f; //in degs

    AutoLib.SquirrelyGyroTimedDriveStep mStep;
    SensorLib.EncoderGyroPosInt mPosInt;	// position integrator

    //constructor
    public PidTuner() {}


    public void init(){
        telemetry.addData("Status", "Initialized");
        bot.init();
        telemetry.addData("TeleOp Init", "");

        localIMU = bot.getImu("mIMU");
        localIMU.setHeadingOffset(initialHeading); //calibrates relative to placement orientation
        PidSetup();

        g1 = new ControllerLib(gamepad1);
        g2 = new ControllerLib(gamepad2);
    }
    @Override
    public void start(){
        gamepad1.setJoystickDeadzone(0.05f);
        gamepad2.setJoystickDeadzone(0.05f);
        bot.start();
        telemetry.addData("TeleOp Start", "");
    }
    @Override
    public void loop() {
        g1.update();
        g2.update();

        if (!lastFaceButton) {
            if (g1.B()) curSelectVar = Range.clip(curSelectVar - 1, 0, 2);
            else if (g1.A()) curSelectVar = Range.clip(curSelectVar + 1, 0, 2);
        }
        lastFaceButton = g1.A() || g1.B();

        //telemetry stuf
        DecimalFormat df = new DecimalFormat("0.000");
        telemetry.addData("Cur Selected Var", curSelectVar);
        telemetry.addData("P_val", df.format(Kp));
        telemetry.addData("I_val", df.format(Ki));
        telemetry.addData("D_val", df.format(Kd));
        telemetry.addData("Increment", PidTuner.Incs.values()[incIndex].inc);

        if (!lastDPad) {
            if (gamepad1.dpad_up) {
                if (curSelectVar == 0) Kp = Kp + PidTuner.Incs.values()[incIndex].inc;
                else if (curSelectVar == 1) Ki = Ki + PidTuner.Incs.values()[incIndex].inc;
                else if (curSelectVar == 2) Kd = Kd + PidTuner.Incs.values()[incIndex].inc;
            } else if (gamepad1.dpad_down) {
                if (curSelectVar == 0) Kp = Kp - PidTuner.Incs.values()[incIndex].inc;
                else if (curSelectVar == 1) Ki = Ki - PidTuner.Incs.values()[incIndex].inc;
                else if (curSelectVar == 2) Kd = Kd - PidTuner.Incs.values()[incIndex].inc;
            } else if (gamepad1.dpad_left)
                incIndex = Range.clip(incIndex - 1, 0, PidTuner.Incs.values().length - 1);
            else if (gamepad1.dpad_right)
                incIndex = Range.clip(incIndex + 1, 0, PidTuner.Incs.values().length - 1);
        }
        lastDPad = gamepad1.dpad_up || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down;

        if (robotSlow && g1.XOnce()) robotSlow = false;
        else if (!robotSlow && g1.XOnce()) robotSlow = true;

        final double deadband = 0.05;
        //right stick controls
        float dx = gamepad1.right_stick_x;
        float dy = -gamepad1.right_stick_y;

        //power = magnitude of dir vector
        double power;
        power = Math.sqrt(dx * dx + dy * dy);
        if (Math.abs(power) < deadband) power = 0; //if we're in the deadzone, don't give power

        scaleInput(power);

        if (robotSlow) {
            power /= 3;
            mStep.setPower((float) power);// set the current power on the step that actually controls the robot
            mStep.setMaxPower((float) 0.3333); // make sure we can rotate even if we're not moving
        }
        else{
            mStep.setPower((float) power);// set the current power on the step that actually controls the robot
            mStep.setMaxPower((float) 1.0); // make sure we can rotate even if we're not moving
        }


        /* the following if statement sets the direction when we're >0 power
         *  Math.atan2 is great and converts rectangular coords to polar
         */
        double dir = 0;
        if (power > 0) {
            dir = Math.atan2(-dx, dy);
            dir *= 180 / Math.PI; //converts radian to degs
            mStep.setDirection((float) dir);
        }

        //left stick controls
        float hx = gamepad1.left_stick_x;
        float hy = -gamepad1.left_stick_y;

        double heading = 0;
        boolean setHeading = false;
        double hMag = Math.sqrt(hx * hx + hy * hy);
        if (hMag > deadband) {
            heading = Math.atan2(-hx, hy);
            heading *= 180 / Math.PI;
            setHeading = true;
        }
        /*
        // also allow inputting of orientation on 8-way pad
        if (gamepad1.dpad_up) { heading = 0; setHeading = true; }
        if (gamepad1.dpad_right) { heading = -90; setHeading = true; }
        if (gamepad1.dpad_down) { heading = 180; setHeading = true; }
        if (gamepad1.dpad_left) { heading = 90; setHeading = true; }
        if (gamepad1.dpad_up && gamepad1.dpad_right) { heading = -45; setHeading = true; }
        if (gamepad1.dpad_down && gamepad1.dpad_right) { heading = -135; setHeading = true; }
        if (gamepad1.dpad_down && gamepad1.dpad_left) { heading = 135; setHeading = true; }
        if (gamepad1.dpad_up && gamepad1.dpad_left) { heading = 45; setHeading = true; }
        */
        if(g1.YOnce()) {
            float imuOff = localIMU.getHeading();
            localIMU.setHeadingOffset(imuOff);
        }
        // set the direction that robot should face on the step that actually controls the robot
        if (setHeading)
            mStep.setHeading((float) heading);
        // run the control step
        mStep.loop();

        telemetry.addData("Power", power);
        telemetry.addData("slow mode", robotSlow);
        telemetry.addData("IMU Heading", localIMU.getHeading());
        pid = new SensorLib.PID((float) Kp, (float) Ki, (float) Kd, (float) KiCutoff);
        mStep = new AutoLib.SquirrelyGyroTimedDriveStep(this, (float)dir, (float)heading, localIMU, pid, bot.getDtMotors(), (float) power, 10000, false);
    }

    @Override
    public void stop(){
        bot.stopAll();
    }

    void PidSetup(){
        pid = new SensorLib.PID((float)Kp, (float)Ki, (float)Kd, (float)KiCutoff);

        mStep = new AutoLib.SquirrelyGyroTimedDriveStep(this, 0, 0, localIMU, pid, bot.getDtMotors(), 0, 10000, false);
        int countsPerRev = (int)Math.round(28*15.6);		// for final gear ratio of 15.6 @ 28 counts/motorRev
        double wheelDiam = 4.0;		    // wheel diameter (in)
        Position initialPos = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);  // example starting position: at origin of field
        SensorLib.EncoderGyroPosInt.DriveType dt = //SensorLib.EncoderGyroPosInt.DriveType.XDRIVE;
                SensorLib.EncoderGyroPosInt.DriveType.MECANUM;
        mPosInt = new SensorLib.EncoderGyroPosInt(dt,this, localIMU, bot.getDtMotors(), countsPerRev, wheelDiam, initialPos);
    }
    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal)  {
        return dVal*dVal*dVal;		// maps {-1,1} -> {-1,1}
    }
}
