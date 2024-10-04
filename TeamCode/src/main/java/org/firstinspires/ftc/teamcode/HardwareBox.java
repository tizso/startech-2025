package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HardwareBox extends LinearOpMode{
    //public NormalizedColorSensor color;

    public DcMotor slider = null;
    public DcMotor arm = null;
    public Servo intake = null;
    public Servo collector = null;

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
        slider = hwMap.get(DcMotor.class, "slider");
        arm = hwMap.get(DcMotor.class, "arm");

        //color = hardwareMap.get(NormalizedColorSensor.class, "color");

        // Set all motors to zero power
        slider.setPower(0.0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
        intake = hwMap.get(Servo.class, "intake");
        collector = hwMap.get(Servo.class, "rightS");

        intake.setPosition(0.0);
        collector.setPosition(0.0);


    }


    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }


    public void sliderUp(){
        slider.setDirection(DcMotorEx.Direction.FORWARD);
        slider.setTargetPosition(240);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(0.7);

        safeWaitSeconds(1);
    }

    public void putSpecimen(){
        safeWaitSeconds(1);
    }

    public void dropSample(){
        safeWaitSeconds(1);
    }


}
