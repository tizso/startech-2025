package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareBox extends LinearOpMode{
    //public NormalizedColorSensor color;

    public DcMotor slider = null;

    //public CRServo collector = null;
    public Servo claw = null;
    public Servo arm = null;

    public Servo rotate = null;
    public Servo sliderServo = null;

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
        slider = hwMap.get(DcMotor.class, "Slider");

        slider.setDirection(DcMotorSimple.Direction.FORWARD);


        slider.setPower(0.0);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //color = hardwareMap.get(NormalizedColorSensor.class, "color");

        // Define and initialize ALL installed servos.
        claw = hwMap.get(Servo.class, "claw");
        arm = hwMap.get(Servo.class, "arm");
        rotate = hwMap.get(Servo.class, "rotate");
        sliderServo = hwMap.get(Servo.class, "slide");


        claw.setPosition(0.55);
        arm.setPosition(0.9);
        rotate.setPosition(0.0);
        sliderServo.setPosition(0.1);
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

    public void moveArm(double a){
        arm.setPosition(a);
    }
    public void moveClaw(double value){
        claw.setPosition(value);
    }
    public void moveSliders(int value, double speed){
        slider.setTargetPosition(value);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(speed);

    }

    public void highChamber(int value){
        slider.setTargetPosition(value);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.8);
        arm.setPosition(0.9);
    }

    public void wall(){

    }

    public void highBasketUp(){
        /*armMotor.setTargetPosition(750);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
        liftMotor.setTargetPosition(1000);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);*/
    }

    public void dropSample(){
        safeWaitSeconds(1);
    }


}
