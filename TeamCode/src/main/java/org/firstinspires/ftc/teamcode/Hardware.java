package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


public class Hardware
{
    /* Public OpMode members. */

    public DcMotor  lF  = null;
    public DcMotor  lB  = null;
    public DcMotor  rF  = null;
    public DcMotor  rB  = null;
    public DcMotorEx shooterL = null;
    public DcMotorEx shooterR = null;
    public DcMotor intake = null;

    //public Servo intakeArmL = null;
    public CRServo feed = null;
    public Servo brake1 = null;

    //public IMU imu;
    public ColorRangeSensor color1;
   // public DistanceSensor armDistance;
    public WebcamName Webcam;
    /* local OpMode members. */
    HardwareMap hwMap =  null;

    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Hardware(){

    }

    /* Initialize standard Techi_Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Techi_Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lF = hwMap.get(DcMotor.class, "lF");
        lB = hwMap.get(DcMotor.class, "lB");
        rF = hwMap.get(DcMotor.class, "rF");
        rB = hwMap.get(DcMotor.class, "rB");
        shooterL = hwMap.get(DcMotorEx.class,"shooterL");
        shooterR = hwMap.get(DcMotorEx.class, "shooterR");
        intake = hwMap.get(DcMotor.class, "intake");

        //intakeArmL = hwMap.get(Servo.class, "intakeArmL");
        feed = hwMap.get(CRServo.class, "feed");
        brake1 = hwMap.get(Servo.class, "brake1");

        color1 = hwMap.get(ColorRangeSensor.class, "color1");
        //armDistance = hwMap.get(DistanceSensor.class, "distance");
        //imu = hwMap.get(IMU.class, "imu");

        lF.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        lB.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rF.setDirection(DcMotor.Direction.FORWARD);
        rB.setDirection(DcMotor.Direction.FORWARD);
        shooterL.setDirection(DcMotorEx.Direction.FORWARD);
        shooterR.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
        shooterL.setPower(0);
        shooterR.setPower(0);
        intake.setPower(0);

        // Set all drive train motors to run without encoders.
        lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


}
