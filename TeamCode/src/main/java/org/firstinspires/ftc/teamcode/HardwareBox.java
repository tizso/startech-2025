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

    public DcMotor rSlider = null;
    public DcMotor lSlider = null;
    //public CRServo collector = null;
    public Servo leftS = null;
    public Servo rightS = null;
    public Servo claw = null;
    public Servo slider = null;

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
        rSlider = hwMap.get(DcMotor.class, "RSlider");
        lSlider = hwMap.get(DcMotor.class, "LSlider");

        rSlider.setDirection(DcMotorSimple.Direction.REVERSE);


        rSlider.setPower(0.0);
        rSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lSlider.setPower(0.0);
        lSlider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //color = hardwareMap.get(NormalizedColorSensor.class, "color");

        /*LSlider.setTargetPosition(0);
        LSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        /*rSlider.setDirection(DcMotorSimple.Direction.REVERSE);
        rSlider.setTargetPosition(0);
        rSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/


        // Define and initialize ALL installed servos.
        leftS = hwMap.get(Servo.class, "leftS");
        rightS = hwMap.get(Servo.class, "rightS");
        claw = hwMap.get(Servo.class, "claw");
        slider = hwMap.get(Servo.class, "slider");

        /*leftS.setPosition(0.05);
        rightS.setPosition(0.05);
        slider.setPosition(0.0);*/
        /*claw.setPosition(0.0);
        slider.setPosition(0.0);*/


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

    public void openArm(){
        /*armMotor.setTargetPosition(750);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.5);
        liftMotor.setTargetPosition(500);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);*/
    }
    public void closeArm(){
        /*armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.3);
        liftMotor.setTargetPosition(5);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);*/
    }

    public void moveSliders(int value){
        rSlider.setTargetPosition(value);
        rSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSlider.setPower(0.5);

        lSlider.setTargetPosition(value);
        lSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSlider.setPower(0.5);
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
