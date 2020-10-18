package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.SensorLib;
//import org.firstinspires.ftc.teamcode.libraries.ToggleButton;

import org.firstinspires.ftc.teamcode.libraries.interfaces.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode.libraries.interfaces.ControllerLib;
import org.firstinspires.ftc.teamcode.libraries.hardware.BotHardware;

@TeleOp(name="Relative IMU Assisted")  // @Autonomous(...) is the other common choice
//@Disabled
public class RelativeIMUassistedTeleOp extends OpMode {

    AutoLib.SquirrelyGyroTimedDriveStep mStep;
    BotHardware bot;

    private BNO055IMUHeadingSensor localIMU = null;

    SensorLib.EncoderGyroPosInt mPosInt;	// position integrator

    /**
     * Constructor
     */
    public RelativeIMUassistedTeleOp() {

    }


    @Override
    public void init() {

        // get hardware
        bot = new BotHardware(this);
        bot.init();

        localIMU = bot.getImu("mIMU");

        // set initial orientation of bot relative to driver (default is 0 degrees == N)
        float initialHeading = 0.0f;	// N
        localIMU.setHeadingOffset(initialHeading);

        // post instructions to console
        /**telemetry.addData("AbsoluteSquirrelyGyroDrive1", "");
        telemetry.addData("left stick", " orientation on field");
        telemetry.addData("dpad", " orientation on field");
        telemetry.addData("right stick", " motion on field");
        telemetry.addData("initial heading", initialHeading);**/

        // construct a PID controller for correcting heading errors
        final float Kp = 0.01f;        // degree heading proportional term correction per degree of deviation
        final float Ki = 0f;        // ... integrator term
        final float Kd = 0f;         // ... derivative term
        final float KiCutoff = 10.0f;   // maximum angle error for which we update integrator
        SensorLib.PID pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        // create a Step that we will use in teleop mode
        mStep = new AutoLib.SquirrelyGyroTimedDriveStep(this, 0, 0, localIMU, pid, bot.getDtMotors(), 0, 10000, false);

        // create Encoder/gyro-based PositionIntegrator to keep track of where we are on the field
        // use constructor that defaults the wheel type to Normal (not Mecanum or X-Drive)
        int countsPerRev = 28*20;		// for 20:1 gearbox motor @ 28 counts/motorRev
        double wheelDiam = 4.0;		    // wheel diameter (in)
        Position initialPosn = new Position(DistanceUnit.INCH, 0.0, 0.0, 0.0, 0);  // example starting position: at origin of field
        SensorLib.EncoderGyroPosInt.DriveType dt = //SensorLib.EncoderGyroPosInt.DriveType.XDRIVE;
                SensorLib.EncoderGyroPosInt.DriveType.MECANUM;
        mPosInt = new SensorLib.EncoderGyroPosInt(dt,this, localIMU, bot.getDtMotors(), countsPerRev, wheelDiam, initialPosn);
    }


    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        gamepad1.setJoystickDeadzone(0.05f);
        gamepad2.setJoystickDeadzone(0.05f);// to avoid zero-stick drifting

        // process gyro correction inputs on lb and rb buttons
        /**lb.process(gamepad1.left_bumper);
        rb.process(gamepad1.right_bumper);
        rh.mIMU.setHeadingOffset(rb.value()-lb.value());**/

        // motion direction is on the right stick
        float dx = gamepad1.right_stick_x;
        float dy = -gamepad1.right_stick_y;	// y is reversed :(

        // power is the magnitude of the direction vector
        double power = Math.sqrt(dx*dx + dy*dy);

        // scale the joystick values to make it easier to control the robot more precisely at slower speeds.
        power = scaleInput(power);

        // set the current power on the step that actually controls the robot
        mStep.setPower((float) power);

        // make sure we can rotate even if we're not moving
        mStep.setMaxPower((float) 1.0);

        // we don't have a valid direction when inputs are "zero"
        if (power > 0) {
            // direction angle of stick >> the direction we want to move
            double direction = Math.atan2(-dx, dy);    // stick angle: zero = +y, positive CCW, range +-pi
            direction *= 180.0 / Math.PI;        // radians to degrees

            // set the direction of motion on the step that actually controls the robot
            mStep.setDirection((float) direction);
        }

        // vehicle heading (orientation) is on the left stick (near the dpad, which also controls heading)
        float hx = gamepad1.left_stick_x;
        float hy = -gamepad1.left_stick_y;    // y is reversed :(

        double heading = 0;
        boolean setHeading = false;
        double hMag = Math.sqrt(hx*hx + hy*hy);
            // direction angle of stick >> the direction we want to face
            heading = Math.atan2(-hx, hy);    // stick angle: zero = +y, positive CCW, range +-pi
            heading *= 180.0 / Math.PI;        // radians to degrees
            setHeading = true;

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
            mStep.setHeading((float) heading);

        // run the control step
        mStep.loop();
    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
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
