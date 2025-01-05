package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareBox extends LinearOpMode{

    double SLIDER_DIFF = 0.956;
    //public NormalizedColorSensor color;

    public DcMotor slider = null;
    public DcMotor slider2 = null;

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
        slider2 = hwMap.get(DcMotor.class, "Slider2");

        slider.setDirection(DcMotorSimple.Direction.FORWARD);
        slider2.setDirection(DcMotorSimple.Direction.REVERSE);


        slider.setPower(0.0);
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slider2.setPower(0.0);
        slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //color = hardwareMap.get(NormalizedColorSensor.class, "color");

        // Define and initialize ALL installed servos.
        claw = hwMap.get(Servo.class, "claw");
        arm = hwMap.get(Servo.class, "arm");
        rotate = hwMap.get(Servo.class, "rotate");
        sliderServo = hwMap.get(Servo.class, "slide");




        claw.setPosition(0.9);
        //claw.setPosition(0.55);
        //arm.setPosition(0.9);
        rotate.setPosition(0.1);
        sliderServo.setPosition(0.2);
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
    public void moveSliders(int v1, double speed){

        int v2 = (int)Math.round(v1*SLIDER_DIFF);

        slider.setTargetPosition(v1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(speed);

        slider2.setTargetPosition(v2);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setPower(speed);

    }

    public void highChamber(int v1){
        int v2 = (int)Math.round(v1*SLIDER_DIFF);

        slider.setTargetPosition(v1);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.8);

        slider2.setTargetPosition(v2);
        slider2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider2.setPower(0.8);
        arm.setPosition(0.9);
    }

    public void wall(){

    }

    public void putSpec(){
        safeWaitSeconds(0.2);
        moveArm(0.5);
        safeWaitSeconds(0.1);
        moveArm(0.3);
        safeWaitSeconds(0.2);
        moveClaw(0.0);


    }


}
