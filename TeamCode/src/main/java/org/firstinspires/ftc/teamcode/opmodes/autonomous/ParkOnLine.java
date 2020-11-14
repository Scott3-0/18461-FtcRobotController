package org.firstinspires.ftc.teamcode.opmodes.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.libraries.AutoLib;
import org.firstinspires.ftc.teamcode.libraries.interfaces.BNO055IMUHeadingSensor;
import org.firstinspires.ftc.teamcode.libraries.SensorLib;
import org.firstinspires.ftc.teamcode.libraries.hardware.BotHardware;

//@Disabled
@Autonomous(name = "Park On Line")
public class ParkOnLine extends OpMode {
    BotHardware bot = new BotHardware(this);
    BNO055IMUHeadingSensor localIMU;
    SensorLib.PID pid;
    //DcMotor motors[];
    //INSERT MOTORS, SERVOS, AND SENSORS HERE
    AutoLib.Sequence seq;
    boolean done;
    float uniPow = 1f;

    @Override
    public void init(){
        bot.init();

        localIMU = bot.getImu("mIMU");
        //localIMU.setHeadingOffset(initialHeading);

        //pid setup stuff
        float Kp = 0.02f;        // motor power proportional term correction per degree of deviation
        float Ki = 0.025f;         // ... integrator term
        float Kd = 0;             // ... derivative term
        float KiCutoff = 10.0f;    // maximum angle error for which we update integrator
        pid = new SensorLib.PID(Kp, Ki, Kd, KiCutoff);

        /*motors = new DcMotor[4];
        motors[0] = bot.fr;
        motors[1] = bot.br;
        motors[2] = bot.fl;
        motors[3] = bot.bl;*/

        //INSERT MOTORS, SERVOS, AND SENSORS HERE

        seq = new AutoLib.LinearSequence();

        //drive forward 75 inches or to (-1.5, 0.5)
        seq.add(new AutoLib.SqPosIntDriveToStep());//TODO Fuck
    }

    @Override
    public void start(){

    }
    @Override
    public void loop(){
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