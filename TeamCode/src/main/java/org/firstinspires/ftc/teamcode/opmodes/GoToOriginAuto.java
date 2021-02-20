package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.SensorLib;
import org.firstinspires.ftc.teamcode.libraries.hardware.BotHardware;
import org.firstinspires.ftc.teamcode.libraries.interfaces.BNO055IMUHeadingSensor;
/*
 *
 * Made by Scott 3.0, 11/13/2020
 *
 */

 //TODO:
    /**Base off of https://github.com/rijdmc419/SkyStone/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/_Auto/DepotSideAuto/ComplexTest.java
     *
     **/
@Autonomous(name="Goto Origin Auto Test")
public class GoToOriginAuto extends OpMode {

    BotHardware bot = new BotHardware(this);
    private BNO055IMUHeadingSensor imu = null;

    float uniPow = bot.fastPow;
    float initHead = 0f;
    SensorLib.PID pid;
    SensorLib.EncoderGyroPosInt posInt; //Encoder/gyro-based position integrator to keep track of where we are
    DcMotor motors[];
    float botLength = 17; //INCHES
    int tl = 24; //Tile length INCHES
    int tol = 2; //(error tolerance) INCHES

    AutoLib.Sequence seq;
    boolean done;

    @Override
    public void init(){
        bot.init();

        imu = bot.getImu("mIMU");
        imu.setHeadingOffset(initHead);

        float initX = 2.5f*tl  -(botLength/2)+3;//2.5f*tl-(botLength/2);
        float initY = -1.25f*tl + 6;//-0.5f*tl-(botLength/2);

        //pid setup stuff
        float Kp = 0.04f;
        float Ki =0.00f;         // ... integrator term
        float Kd = 0f;      // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        motors = new DcMotor[4];
        motors[0] = BotHardware.Motor.frontRight.motor;
        motors[1] =BotHardware.Motor.backRight.motor;
        motors[2] = BotHardware.Motor.frontLeft.motor;
        motors[3] = BotHardware.Motor.backLeft.motor;

        for(int i = 0; i < motors.length; i++){
            motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        Position initPos = new Position(DistanceUnit.INCH, initX, initY, 0, 0);
        posInt = new SensorLib.EncoderGyroPosInt(SensorLib.EncoderGyroPosInt.DriveType.MECANUM, this, imu, motors, 560, 3.937f, initPos);

        seq = new AutoLib.LinearSequence();

        seq.add(new AutoLib.SqPosIntDriveToStep(this, posInt, motors, uniPow, pid, new Position(DistanceUnit.INCH, -0.25*tl, -1.25*tl, 0, 0), 90, tol, true));
    }
    @Override
    public void start(){

    }
    @Override
    public void loop(){
        telemetry.addData("", motors[0].getPower() + motors[1].getPower() + motors[2].getPower() + motors[3].getPower());
        if (!done){
            done = seq.loop(); // returns true when we're done
        }
        else{
            telemetry.addData("Sequence finished", "");
        }
    }
    @Override
    public void stop(){
        super.stop();
    }
}
