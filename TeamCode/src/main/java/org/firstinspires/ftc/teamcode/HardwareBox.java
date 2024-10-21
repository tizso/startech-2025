package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class HardwareBox extends LinearOpMode{
    //public NormalizedColorSensor color;

    public DcMotor armMotor = null;
    public DcMotor liftMotor = null;
    public CRServo collector = null;
    //public Servo collector = null;

    HardwareMap hwMap		   =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareBox(){

    }
    public void runOpMode(){}

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        liftMotor = hwMap.get(DcMotor.class, "liftMotor");
        armMotor = hwMap.get(DcMotor.class, "armMotor");


        armMotor.setPower(0.0);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //color = hardwareMap.get(NormalizedColorSensor.class, "color");

/*        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        /*liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/


        // Define and initialize ALL installed servos.
        collector = hwMap.get(CRServo.class, "collector");
        //collector = hwMap.get(Servo.class, "rightS");

        collector.setPower(0.0);
        //collector.setPosition(0.0);


    }


    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


   /* public void sliderUp(){
        slider.setDirection(DcMotorEx.Direction.FORWARD);
        slider.setTargetPosition(240);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.7);

        safeWaitSeconds(1);
    }*/

    public void putSpecimen(){
        safeWaitSeconds(1);
    }

    public void dropSample(){
        safeWaitSeconds(1);
    }


}
