package org.firstinspires.ftc.teamcode.libraries.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libraries.interfaces.BNO055IMUHeadingSensor;

/**
 * Made by Scott 3.0 10/4/2020
 *  hardware = yes.
 */
public class BotHardware {
    //def not stealing Noah's idea of enums
    public float slowPow = 0.33f;
    public float fastPow = 1f;

    public enum Motor {
        frontRight("fr", false),
        backRight("br", false),
        frontLeft("fl", true),
        backLeft("bl", true),
        out("outtake", false);
        private final String name;
        private final boolean reverse;
        public DcMotorEx motor;

        Motor(String name, boolean reverse) {
            this.name = name;
            this.reverse = reverse;
        }

        void initMotor(OpMode mode) {
            try {
                this.motor = mode.hardwareMap.get(DcMotorEx.class, this.name);
                this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (this.reverse)
                    this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            catch (Exception e) {
                mode.telemetry.addData(this.name, "Failed to find motor");
            }
        }
    } //motor enum close brack
     public enum ServoE {
        out("outServo", true),
        arm("sensorArm", false);

        public static final double outL = 0.53;
        public static final double outR = 0.2;

        public static final double armL = 0.45;
        public static final double armR = 0.07;

        private final String name;
        public Servo servo;
        private boolean reversed;

        ServoE(String name, boolean reversed){
            this.reversed = reversed;
            this.name = name;
        }

        void initServo(OpMode mode){
            try{
                this.servo = mode.hardwareMap.get(Servo.class, this.name);
            }
            catch(Exception e){
                mode.telemetry.addData(this.name, "Failed to find servo");
            }
        }
    } //servo enum close brack

     public enum Imu {
         mIMU("imu", 4);

         private final String name;
         private final int orientation;
         public BNO055IMUHeadingSensor imu;

         Imu(String name, int orientation) {
             this.name = name;
             this.orientation = orientation;
         }

         void initImu(OpMode mode) {
             try {
                 this.imu = new BNO055IMUHeadingSensor(mode.hardwareMap.get(BNO055IMU.class, this.name));
                 this.imu.init(4);
             }
             catch (Exception e) {
                 mode.telemetry.addData(this.name, "Failed to find motor");
             }
         }
     }
    //opmode pointer
    private final OpMode mode;

    public BotHardware(OpMode mode){
        this.mode = mode;
    }
    public void init(){
        //motor init
        for(int i = 0; i < Motor.values().length; i++){
            Motor.values()[i].initMotor(this.mode);
        }
        //servo init
        for(int i = 0; i < ServoE.values().length; i++){
            ServoE.values()[i].initServo(this.mode);
        }
        //IMU init
        Imu.mIMU.initImu(this.mode);
    }

    public void start(){

    }

    public void setFrDrive(double power) { Motor.frontRight.motor.setPower(power); }

    public void setBrDrive(double power) {
        Motor.backRight.motor.setPower(power);
    }

    public void setFlDrive(double power) {
        Motor.frontLeft.motor.setPower(power);
    }

    public void setBlDrive(double power) {
        Motor.backLeft.motor.setPower(power);
    }

    public void setOutPower(double power){Motor.out.motor.setPower(power);}
    //ORDER IS IMPORTANT
    //FR, BR, FL, BL
    //DON'T FUCK IT UP
    public void setAllDrive(double FRpower, double BRpower, double FLpower, double BLpower){
        Motor.frontRight.motor.setPower(FRpower);
        Motor.backRight.motor.setPower(BRpower);
        Motor.frontLeft.motor.setPower(FLpower);
        Motor.backLeft.motor.setPower(BLpower);
    }


    public void stopAll() {
        for(Motor motor : Motor.values()) motor.motor.setPower(0);
    }

    public DcMotorEx getMotor(String name) {
        return Motor.valueOf(name).motor;
    }
    public DcMotorEx[] getDtMotors(){
        //ORDER IS IMPORTANT
        //FR, BR, FL, BL
        //DON'T FUCK IT UP
        DcMotorEx[] DtMotors={Motor.frontRight.motor, Motor.backRight.motor, Motor.frontLeft.motor, Motor.backLeft.motor};
        return DtMotors;

    }
    public BNO055IMUHeadingSensor getImu(String name){
        return Imu.valueOf(name).imu;
    }
}
