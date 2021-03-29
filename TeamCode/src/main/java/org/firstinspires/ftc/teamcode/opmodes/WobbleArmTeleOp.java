package org.firstinspires.ftc.teamcode.opmodes;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.SensorLib;
import org.firstinspires.ftc.teamcode.libraries.hardware.BotHardware;
import org.firstinspires.ftc.teamcode.libraries.interfaces.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode.libraries.interfaces.ControllerLib;
import org.firstinspires.ftc.teamcode.libraries.StupidMotorLib;

/**
 *
 * Made by Daniel, 03/17/21
 *
 **/
//@Disabled
@TeleOp(name="Wobble Arm TeleOp")
public class WobbleArmTeleOp extends OpMode {
    private static final float slowPow = 0.33f;
    private static final float fastPow = 1.0f;
    private boolean robotSlow = false; //init value
    private boolean lastA = true;

    private ControllerLib g1;
    private ControllerLib g2;

    private DcMotor StupidWobble;

    protected BotHardware bot = new BotHardware(this);

    private BNO055IMUHeadingSensor localIMU = null;
    float initialHeading = 90f; //in DEG //TODO: See if needs changed
    /*
    *          [-90]
    *           |
    *           |
    *  [0] ----@---- [+/- 180]
    *           |
    *           |
    *         [90]
     */

    AutoLib.SquirrelyGyroTimedDriveStep mStep;
    SensorLib.EncoderGyroPosInt mPosInt;	// position integrator

    //constructor
    public WobbleArmTeleOp() {}

    @Override
    public void init(){
        bot.init();
        telemetry.addData("TeleOp Init", "");

        localIMU = bot.getImu("mIMU");
        localIMU.setHeadingOffset(initialHeading); //calibrates relative to placement orientation
        PidSetup();

        g1 = new ControllerLib(gamepad1);
        g2 = new ControllerLib(gamepad2);

        StupidWobble = bot.getMotor("wobble");

    }
    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void start(){
        gamepad1.setJoystickDeadzone(0.05f);
        gamepad2.setJoystickDeadzone(0.05f);
        bot.start();
        telemetry.addData("TeleOp Start", "");
        //StupidWobble.setPosition(250); //TODO FIX
    }

    @Override
    public  void loop(){
        g1.update();
        g2.update();
        if(robotSlow && g1.leftBumperOnce()) robotSlow = false;
        else if(!robotSlow && g1.leftBumperOnce()) robotSlow = true;

        final double deadband = 0.75; //this deadzones the rotation stick (set high)
        //left stick controls
        float dx = gamepad1.right_stick_x;
        float dy = -gamepad1.right_stick_y;

        //power = magnitude of dir vector
        double power = Math.sqrt(dx*dx + dy*dy);
        //if(Math.abs(power) < deadband) power = 0; //if we're in the deadzone, don't give power

        if(robotSlow) power = (0.33f) * scaleInput(power); // cube the joystick values to make it easier to control the robot more precisely at slower speeds
        else if(!robotSlow) power = scaleInput(power);
        mStep.setPower((float) power);// set the current power on the step that actually controls the robot
        mStep.setMaxPower(0.85f); // make sure we can rotate even if we're not moving

         /* the following if statement sets the direction when we're >0 power
          *  Math.atan2 is great and converts rectangular coords to polar
          */
        if(power > 0){
            double dir = Math.atan2(-dx, dy);
            dir *= 180 /Math.PI; //converts radian to degs
            mStep.setDirection((float) dir);
        }

        //right stick controls
        float hx = gamepad1.left_stick_x;
        float hy = -gamepad1.left_stick_y;

        double heading = 0;
        boolean setHeading = false;
        double hMag = Math.sqrt(hx*hx + hy*hy);
        if(hMag > deadband){
            heading = Math.atan2(-hx, hy);
            heading *= 180 / Math.PI; //rad 2 deg
            setHeading = true;
        }

        // also allow inputting of orientation on 8-way pad
        if (gamepad1.dpad_up) { heading = 0; setHeading = true; }
        if (gamepad1.dpad_right) { heading = -90; setHeading = true; }
        if (gamepad1.dpad_down) { heading = 180; setHeading = true; }
        if (gamepad1.dpad_left) { heading = 90; setHeading = true; }
        if (gamepad1.dpad_up && gamepad1.dpad_right) { heading = -45; setHeading = true; }
        if (gamepad1.dpad_down && gamepad1.dpad_right) { heading = -135; setHeading = true; }
        if (gamepad1.dpad_down && gamepad1.dpad_left) { heading = 135; setHeading = true; }
        if (gamepad1.dpad_up && gamepad1.dpad_left) { heading = 45; setHeading = true; }


        // set the direction that robot should face on the step that actually controls the robot
        if (setHeading)
            mStep.setHeading((float)heading); //TODO: see if 90 breaks it

        // run the control step
        mStep.loop();

        telemetry.addData("Moto Pow", power);
        telemetry.addData("slow mode", robotSlow);
         telemetry.addData("IMU Heading", localIMU.getHeading());
         telemetry.addData("ArmServ",BotHardware.ServoE.arm.servo.getPosition());
         telemetry.addData("armPow", StupidWobble.getPower());
         //telemetry.addData("StupidWobble", StupidWobble.getCounts());
        /**
         * Unfinished OpMode
         * TODO: Finish the IMU integration
         * See: https://github.com/rijdmc419/SkyStone/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/_TeleOp/AbsoluteSquirrelyGyroDrive1PUAL.java#L76
        **/

        if(g1.rightBumperOnce()){
            BotHardware.ServoE.out.servo.setPosition(BotHardware.ServoE.outR);
            SystemClock.sleep(250);
            BotHardware.ServoE.out.servo.setPosition(BotHardware.ServoE.outL);

        }

        if(g1.right_trigger>0.5f){
            bot.setOutPower(1);
        }
        else {
            bot.setOutPower(0);
        }

        if(g2.dpadUp()){
            StupidWobble.setPower(-1); //TODO FIX
        }
        else if(g2.dpadDown()){
            StupidWobble.setPower(0.66); //TODO FIX
        }
        else{
            StupidWobble.setPower(0);
        }

        if(g2.right_trigger>0.5f){
            BotHardware.ServoE.arm.servo.setPosition(0); //TODO FIX
        }
        else{
            BotHardware.ServoE.arm.servo.setPosition(1); //TODO FIX
        }
    }

    //PID Constructor
    void PidSetup(){
        // construct a PID controller for correcting heading errors
        final float Kp = 0.05f;        // degree heading proportional term correction per degree of deviation
        final float Ki = 0.10f;        // ... integrator term 0.09
        final float Kd = 0f;         // ... derivative term 0.001
        final float KiCutoff = 7.5f;   // maximum angle error for which we update integrator
        SensorLib.PID pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        mStep = new AutoLib.SquirrelyGyroTimedDriveStep(this, 0, 90, localIMU, pid, bot.getDtMotors(), 0, 10000, false);
        int countsPerRev = (int)Math.round(28*15.6);		// for final gear ratio of 15.6 @ 28 counts/motorRev
        double wheelDiam = 4.0;		    // wheel diameter (in)
        Position initialPos = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);  // example starting position: at origin of field
        SensorLib.EncoderGyroPosInt.DriveType dt = //SensorLib.EncoderGyroPosInt.DriveType.XDRIVE;
                SensorLib.EncoderGyroPosInt.DriveType.MECANUM;
        mPosInt = new SensorLib.EncoderGyroPosInt(dt,this, localIMU, bot.getDtMotors(), countsPerRev, wheelDiam, initialPos);
    }

    @Override
    public void stop(){
        bot.stopAll();
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
