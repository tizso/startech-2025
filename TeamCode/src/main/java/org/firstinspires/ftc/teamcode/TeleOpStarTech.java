package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;


@TeleOp(name = "TeleOp StarTech - 2025", group="00-TeleOp")
public class TeleOpStarTech extends LinearOpMode {
    HardwareBox robot = new HardwareBox();

    final double ARM_TICKS_PER_DEGREE = 28*94329.0/4913.0*100.0/20.0*1/360;

    final double LIFT_TICKS_PER_MM = (111132.0 / 289.0) / 120.0;

    final double LIFT_COLLAPSED = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_LOW_BASKET = 0 * LIFT_TICKS_PER_MM;
    final double LIFT_SCORING_IN_HIGH_BASKET = 480 * LIFT_TICKS_PER_MM;

    double liftPosition = LIFT_COLLAPSED;

    double cycletime = 0;
    double looptime = 0;
    double oldtime = 0;

    double armLiftComp = 0;

    double SLOW_DOWN_FACTOR = 0.9; //TODO Adjust to driver comfort
    boolean changed = false;
    boolean slow = false;
    boolean rotate = false;
    boolean arm = false;
    boolean claw = false;
    boolean spec = false;

    double pos = 0;
    int sliderPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        robot.init(hardwareMap);


        telemetry.addData("Initializing FTC TeleOp adopted for Team:","18338");
        telemetry.update();


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
            double sliderSpeed = 0.7;
            if(currentGamepad1.a && !previousGamepad1.a){
                slow = !slow;
            } else if (gamepad1.b){

                robot.moveSliders(0, sliderSpeed);
            }

            else if (gamepad1.x){
                robot.moveSliders(1800, sliderSpeed);
            } else if(gamepad1.y){
                robot.moveSliders(4100, sliderSpeed);
            }
            else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                //armPosition = ARM_COLLAPSED_INTO_ROBOT;
                //liftPosition = LIFT_COLLAPSED;
            }
            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                //armPosition = ARM_SCORE_SPECIMEN;
            }
            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                //armPosition = ARM_ATTACH_HANGING_HOOK;
            }
            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                //armPosition = ARM_WINCH_ROBOT;
            } else if (gamepad1.left_bumper) {


            } else if (gamepad1.right_bumper) {


            }

            SLOW_DOWN_FACTOR = slow?0.3:0.9;

           /* robot.armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
            ((DcMotorEx) robot.armMotor).setVelocity(2100);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

            if(currentGamepad2.a && !previousGamepad2.a){
                rotate = !rotate;
            } else if(currentGamepad2.b && !previousGamepad2.b){
                arm = !arm;
            } else if(currentGamepad2.x && !previousGamepad2.x){
                spec = !spec;
            } else if(gamepad2.y && !changed){
                changed = true;
            } else if(gamepad2.back){
                sleep(200);
            } else if(currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                claw = !claw;
            } else if (gamepad2.left_bumper) {
                sleep(200);
            }

            if(rotate){
                robot.rotate.setPosition(1.0);
            } else {
                robot.rotate.setPosition(0.0);
            }

            if(arm){
                robot.arm.setPosition(0.5);
            } else if(spec){
                robot.arm.setPosition(0.9);
            } else {
                robot.arm.setPosition(0.0);
            }

            if(claw){
                robot.claw.setPosition(0.65);

            } else {
                robot.claw.setPosition(0.9);
            }

            if(robot.slider.getCurrentPosition()<100 && !gamepad1.x && !gamepad1.y){
                robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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


            telemetry.addLine("Current Pose");
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
            telemetry.addData("Slider", robot.slider.getCurrentPosition());
            telemetry.addData("claw", robot.claw.getPosition());
            telemetry.addData("gamepad1.x", gamepad1.x);

            telemetry.update();
        }
    }
}
