package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "TeleOp StarTech - 2025", group="00-TeleOp")
public class TeleOpStarTech extends LinearOpMode {
    HardwareBox robot = new HardwareBox();

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double SLOW_DOWN_FACTOR = 0.9; //TODO Adjust to driver comfort
    boolean slow = false;
    boolean rotate = false;
    boolean arm = false;
    boolean claw = false;
    boolean spec = false;
    boolean hb = false;
    boolean sd = false;
    boolean sliderServo = false;
    double servoInitPosition = 0.2;
    double servoEndPosition = 0.55;
    double servoSensitivity = 0.1;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        robot.init(hardwareMap);


        telemetry.addData("Initializing FTC TeleOp adopted for Team:","18338");
        telemetry.update();

        robot.arm.setPosition(0.0);

        /**
         * If we use joystick to control the servo slider
         */
        //robot.sliderServo.setDirection(Servo.Direction.REVERSE);


        sleep(200);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            telemetry.addData("Running StarTech TeleOp Mode adopted for Team:","18338");
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                            -gamepad1.left_stick_x * SLOW_DOWN_FACTOR
                    ),
                    -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
            ));
            drive.updatePoseEstimate();
            double sliderSpeed = 0.9;
            if(currentGamepad1.a && !previousGamepad1.a){
                slow = !slow;

            } else if (gamepad1.b){
                robot.moveSliders(200, sliderSpeed);
                //robot.moveSliders(300, 285, sliderSpeed);

            }

            else if (gamepad1.x){
                robot.moveSliders(945, sliderSpeed);
                //robot.moveSliders(1800, 1725,sliderSpeed);
            } else if(gamepad1.y){
                robot.moveSliders(2770, sliderSpeed);
                //robot.moveSliders(3900, 3770, sliderSpeed);
            }
            else if(gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                //armPosition = ARM_COLLAPSED_INTO_ROBOT;
                //liftPosition = LIFT_COLLAPSED;
                sleep(500);
            }
            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                //armPosition = ARM_SCORE_SPECIMEN;
                robot.robotUpDown(15000, 0.9);
            }
            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                //armPosition = ARM_ATTACH_HANGING_HOOK;
                robot.slideragatareUp(850);
            }
            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                //armPosition = ARM_WINCH_ROBOT;
                robot.slideragatareUp(0);
            } else if (gamepad1.left_bumper) {
                sleep(500);

            } else if (gamepad1.right_bumper) {
                robot.moveSliders(0, 0.9);
            }

            SLOW_DOWN_FACTOR = slow?0.3:0.9;

            if(currentGamepad2.a && !previousGamepad2.a){
                rotate = !rotate;
            } else if(currentGamepad2.b && !previousGamepad2.b){
                arm = !arm;
            } else if(currentGamepad2.x && !previousGamepad2.x){
                spec = !spec;
            } else if(currentGamepad2.y && !previousGamepad2.y){
                hb = !hb;
            } else if(gamepad2.back){
                sleep(200);
            } else if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                claw = !claw;
            } else if(currentGamepad2.back && !previousGamepad2.back){
                sd = !sd;
            } else if(gamepad2.dpad_down){
                robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slider2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slider.setPower(-gamepad2.right_stick_y);
                robot.slider2.setPower(-gamepad2.right_stick_y);
            } else if(gamepad2.dpad_up){
                robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if(rotate){
                robot.rotate.setPosition(1.0);
            } else {
                robot.rotate.setPosition(0.1 );
            }

            if(arm){
                robot.arm.setPosition(0.5);
            } else if(spec){
                robot.arm.setPosition(0.9);
            } else if(hb){
                robot.arm.setPosition(0.75);
            } else {
                robot.arm.setPosition(0.0);
            }

            if(claw){
                robot.claw.setPosition(0.55);
            } else {
                robot.claw.setPosition(1);
            }

            if(sd) {

            }

            /*if(sliderServo){
                robot.sliderServo.setPosition(1.0);
            } else {
                robot.sliderServo.setPosition(0.1);
            }*/

            double servoPosition = servoInitPosition - (gamepad2.left_stick_y);
            robot.sliderServo.setPosition(servoPosition);
            /*if(servoPosition >= servoEndPosition){
                robot.sliderServo.setPosition(0.55);
            } else if(servoPosition <= servoInitPosition){
                robot.sliderServo.setPosition(0.1);
            } else {

            }*/

            if((robot.slider2.getCurrentPosition()<150 && !gamepad1.x && !gamepad1.y && !gamepad1.b) || gamepad1.left_bumper){
                robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;

            Gamepad.LedEffect rgbEffect = new Gamepad.LedEffect.Builder()
                    .addStep(1,0,0,250)
                    .addStep(0,1,0,250)
                    .addStep(0,0,1,250)
                    .addStep(1,1,1,250)
                    .build();


            /*telemetry.addLine("Current Pose");
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));*/
            telemetry.addData("Slider", robot.slider.getCurrentPosition());
            telemetry.addData("Slider2", robot.slider2.getCurrentPosition());
            telemetry.addData("servoPosition", servoPosition);
            telemetry.addData("sd", sd);
            telemetry.addData("right_stick_y", gamepad2.right_stick_y);
            telemetry.addData("gamepad2.dpad_down", gamepad2.dpad_down);
            /*telemetry.addData("claw", robot.claw.getPosition());
            telemetry.addData("gamepad1.x", gamepad1.x);*/

            telemetry.update();
        }
    }
}

