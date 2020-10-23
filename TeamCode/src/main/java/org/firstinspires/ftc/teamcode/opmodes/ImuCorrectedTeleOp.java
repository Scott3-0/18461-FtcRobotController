package org.firstinspires.ftc.teamcode.opmodes;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.SensorLib;
import org.firstinspires.ftc.teamcode.libraries.hardware.BotHardware;
import org.firstinspires.ftc.teamcode.libraries.interfaces.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode.libraries.interfaces.ControllerLib;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.sql.Time;

/**
 *
 * Made by Scott 3.0, 10/11/2020
 *
 **/
@Disabled
@TeleOp(name="IMU Corrected TeleOp")
public class ImuCorrectedTeleOp extends OpMode {
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
    public ImuCorrectedTeleOp() {}

    @Override
    public void init(){
        bot.init();
        telemetry.addData("TeleOp Init", "");

        localIMU = bot.getImu("mIMU");
        localIMU.setHeadingOffset(initialHeading); //calibrates relative to placement orientation
        PidSetup();

        g1 = new ControllerLib(gamepad1);
        g2 = new ControllerLib(gamepad2);
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
    }

    @Override
    public  void loop(){
        if(robotSlow && gamepad1.a){
            robotSlow = false;
            SystemClock.sleep(500);

        }
        else if(!robotSlow && gamepad1.a){
            robotSlow = true;
            SystemClock.sleep(500);
        }

        final double deadband = 0.05;
        //left stick controls
        float dx = gamepad1.right_stick_x;
        float dy = -gamepad1.right_stick_y;

        //power = magnitude of dir vector
        double power = Math.sqrt(dx*dx + dy*dy);
        if(Math.abs(power) < deadband) power = 0; //if we're in the deadzone, don't give power

        power = scaleInput(power); // cube the joystick values to make it easier to control the robot more precisely at slower speeds
        mStep.setPower((float) power);// set the current power on the step that actually controls the robot
        mStep.setMaxPower((float) 1.0); // make sure we can rotate even if we're not moving

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

        // set the direction that robot should face on the step that actually controls the robot
        if (setHeading)
            mStep.setHeading((float) heading);

        // run the control step
        mStep.loop();
        /**//**//**//**//**
        float tx = 2*gamepad1.right_stick_x*gamepad1.right_stick_x*gamepad1.right_stick_x; // WTF??
        float ty = -1*gamepad1.left_stick_y*gamepad1.left_stick_y*gamepad1.left_stick_y;
        float left = (ty +tx /2 );
        float right = (ty -tx/2);

        float x = g1.left_stick_x*g1.left_stick_x*g1.left_stick_x; //strafe
        float y = -1*g1.right_stick_y*g1.right_stick_y*g1.right_stick_y;//forward & back

        double theta = Math.atan2(-x, y);
        double heading = theta * 180.0/Math.PI;

        AutoLib.MotorPowers mp = AutoLib.GetSquirrelyWheelMotorPowers(heading);
        double front = mp.Front();
        double back = mp.Back();
        double power = Math.sqrt(x*x + y*y);
        front *= power;
        back *= power;

        if(robotSlow && g1.AOnce()){
            robotSlow = false;
            //SystemClock.sleep(500);

        }
        else if(!robotSlow && g1.AOnce()){
            robotSlow = true;
            //SystemClock.sleep(500);
        }

        double fr = 0f;
        double br = 0f;
        double fl = 0f;
        double bl = 0f;

        if(robotSlow) {
            fr = (back + right) * slowPow; //switched right sides font and back
            br = (front + right) * slowPow; // WTF??
            fl = (front + left) * slowPow;
            bl = (back + left) * slowPow;
        }
        else if(!robotSlow){
            fr = (back + right) * fastPow; //switched right sides font and back
            br = (front + right) * fastPow; // WTF??
            fl = (front + left) * fastPow;
            bl = (back + left) * fastPow;
        }

        fr = Range.clip(fr, -1, 1);
        br = Range.clip(br, -1, 1);
        fl = Range.clip(fl, -1, 1);
        bl = Range.clip(bl, -1, 1);
        bot.setAllDrive(fr, br, fl, bl);

        telemetry.addData("Moto Pow", fr + ", " + br + ", " + fl +", " + bl);
        telemetry.addData("slow mode", robotSlow);
        /**//**//**//**//**
        telemetry.addData("IMU Heading", localIMU.getHeading());
        mStep.setPower((float) power);
        mStep.setMaxPower((float) 1.0);

        if(power > 0){
            d
        }
        **//**//**//**//**
        mStep.loop(); */
        /**
         * Unfinished OpMode
         * TODO: Finish the IMU integration
         * See: https://github.com/rijdmc419/SkyStone/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/_TeleOp/AbsoluteSquirrelyGyroDrive1PUAL.java#L76
        **/
    }

    //PID Constructor
    void PidSetup(){
        // construct a PID controller for correcting heading errors
        final float Kp = 0.01f;        // degree heading proportional term correction per degree of deviation
        final float Ki = 0f;        // ... integrator term
        final float Kd = 0f;         // ... derivative term
        final float KiCutoff = 10.0f;   // maximum angle error for which we update integrator
        SensorLib.PID pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        mStep = new AutoLib.SquirrelyGyroTimedDriveStep(this, 0, 0, localIMU, pid, bot.getDtMotors(), 0, 10000, false);
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
