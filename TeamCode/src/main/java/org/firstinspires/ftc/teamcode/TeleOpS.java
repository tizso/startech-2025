package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@TeleOp(name = "TeleOp StarTech - 2025", group="00-TeleOp")
public class TeleOpS extends LinearOpMode {

    HardwareBox robot = new HardwareBox();
    /*final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0;*/
    final double ARM_TICKS_PER_DEGREE = 28*94329.0/4913.0*100.0/20.0*1/360;
    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 0 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 15 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 90 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 110 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 10  * ARM_TICKS_PER_DEGREE;

    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

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

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) robot.armMotor).setCurrentAlert(5, CurrentUnit.AMPS);

        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftMotor.setTargetPosition(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Initializing FTC TeleOp adopted for Team:","18338");
        telemetry.update();


        sleep(200);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            /*if(gamepad1.b ){
                SLOW_DOWN_FACTOR = 0.3;
            }
            if(gamepad1.a ){
                SLOW_DOWN_FACTOR = 0.9;
            }
*/
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

            /*if(gamepad1.dpad_up && up == 0){
                robot.slider.setDirection(DcMotorEx.Direction.FORWARD);
                robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slider.setPower(sliderSpeed/4);
                up=1;
                sleep(200);
            }

            if(gamepad1.dpad_up && up == 1){
                robot.slider.setPower(0);
                up = 0;
                sleep(200);
            }

            if(gamepad1.dpad_down & down == 0){
                robot.slider.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.slider.setPower(sliderSpeed/4);
                down = 1;
                sleep(200);
            }

            if(gamepad1.dpad_down & down == 1){
                robot.slider.setPower(0);
                down = 0;
                sleep(200);
            }
*/

            if (gamepad1.left_bumper) {
                robot.collector.setPower(INTAKE_COLLECT);
            } else if (gamepad1.right_bumper) {
                robot.collector.setPower(INTAKE_OFF);
            } else if(gamepad1.y || gamepad1.x){
                robot.collector.setPower(INTAKE_DEPOSIT);
            }

            armPositionFudgeFactor = FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            if(gamepad1.a){
                /* This is the intaking/collecting arm position */
                armPosition = ARM_COLLECT;
                //liftPosition = LIFT_COLLAPSED;

                robot.collector.setPower(INTAKE_COLLECT);
            }

            else if (gamepad1.b){
                    /* This is about 20° up from the collecting position to clear the barrier
                    Note here that we don't set the wrist position or the intake power when we
                    select this "mode", this means that the intake and wrist will continue what
                    they were doing before we clicked left bumper. */
                armPosition = ARM_CLEAR_BARRIER;
            }

            else if (gamepad1.x){
                /* This is the correct height to score the sample in the HIGH BASKET */
                armPosition = ARM_SCORE_SAMPLE_IN_LOW;
                //liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }

            else if (gamepad1.dpad_left) {
                    /* This turns off the intake, folds in the wrist, and moves the arm
                    back to folded inside the robot. This is also the starting configuration */
                armPosition = ARM_COLLAPSED_INTO_ROBOT;
                //liftPosition = LIFT_COLLAPSED;
                robot.collector.setPower(INTAKE_OFF);

            }

            else if (gamepad1.dpad_right){
                /* This is the correct height to score SPECIMEN on the HIGH CHAMBER */
                armPosition = ARM_SCORE_SPECIMEN;

            }

            else if (gamepad1.dpad_up){
                /* This sets the arm to vertical to hook onto the LOW RUNG for hanging */
                armPosition = ARM_ATTACH_HANGING_HOOK;
                robot.collector.setPower(INTAKE_OFF);

            }

            else if (gamepad1.dpad_down){
                /* this moves the arm down to lift the robot up once it has been hooked */
                armPosition = ARM_WINCH_ROBOT;
                robot.collector.setPower(INTAKE_OFF);

            }

            if (armPosition < 45 * ARM_TICKS_PER_DEGREE){
                armLiftComp = (0.25568 * liftPosition);
            }
            else{
                armLiftComp = 0;
            }

            robot.armMotor.setTargetPosition((int) (armPosition + armPositionFudgeFactor + armLiftComp));
            ((DcMotorEx) robot.armMotor).setVelocity(2100);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad2.a){
                /*robot.arm.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.arm.setTargetPosition(100);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.5);*/
                sleep(200);
            }


            else if(gamepad2.b){
                sleep(200);
            }

            else if(gamepad2.x){
                sleep(200);
            }

            else if(gamepad2.y){
                sleep(200);
            }

            else if(gamepad2.back){
                sleep(200);
            }


            else if (gamepad1.right_bumper) {
                liftPosition += 2800 * cycletime;
                sleep(300);
            }

            else if (gamepad1.left_bumper) {
                liftPosition -= 2800 * cycletime;
                sleep(300);
            }

            if (liftPosition > LIFT_SCORING_IN_HIGH_BASKET){
                liftPosition = LIFT_SCORING_IN_HIGH_BASKET;
            }

            if (liftPosition < 0){
                liftPosition = 0;
            }


            robot.liftMotor.setTargetPosition((int) (liftPosition));

            ((DcMotorEx) robot.liftMotor).setVelocity(2100);
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            looptime = getRuntime();
            cycletime = looptime-oldtime;
            oldtime = looptime;


            telemetry.addLine("Current Pose");
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
            telemetry.addData("armPosition", robot.armMotor.getCurrentPosition());
            telemetry.addData("ARM_TICKS_PER_DEGREE", ARM_TICKS_PER_DEGREE);
            telemetry.addData("lift variable", liftPosition);
            telemetry.addData("Lift Target Position",robot.liftMotor.getTargetPosition());
            telemetry.addData("lift current position", robot.liftMotor.getCurrentPosition());
            telemetry.addData("liftMotor Current:",((DcMotorEx) robot.liftMotor).getCurrent(CurrentUnit.AMPS));
           /* telemetry.addData("slider position",robot.slider.getCurrentPosition());
            telemetry.addData("srm", robot.arm.getCurrentPosition());
            telemetry.addData("collector", robot.collector.getPosition());
            telemetry.addData("intake: ", robot.intake.getDirection());*/

            telemetry.update();
        }
    }
}

