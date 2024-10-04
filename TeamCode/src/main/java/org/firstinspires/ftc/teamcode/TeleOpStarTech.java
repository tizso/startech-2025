package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOp StarTech - 2025", group="00-TeleOp")
public class TeleOpStarTech extends LinearOpMode {

    HardwareBox robot = new HardwareBox();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        double SLOW_DOWN_FACTOR = 0.9; //TODO Adjust to driver comfort
        telemetry.addData("Initializing FTC TeleOp adopted for Team:","18338");
        telemetry.update();

        robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.slider.setDirection(DcMotorEx.Direction.FORWARD);
        robot.slider.setTargetPosition(0);
        robot.slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slider.setPower(0.9);
        sleep(200);

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.b ){
                SLOW_DOWN_FACTOR = 0.3;
            }
            if(gamepad1.a ){
                SLOW_DOWN_FACTOR = 0.9;
            }

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
            if(gamepad1.y || gamepad1.x){
                sleep(200);
            }

            if (gamepad1.left_bumper) {
                sleep(300);
            }

            if (gamepad1.right_bumper) {
                sleep(300);
            }

            if(gamepad1.back){
                sleep(300);
            }

            if(gamepad2.a){
                robot.arm.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.arm.setTargetPosition(100);
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.5);
                sleep(200);
            }

            if(gamepad2.b){
                sleep(200);
            }

            if(gamepad2.x){
                sleep(200);
            }

            if(gamepad2.y){
                sleep(200);
            }

            if(gamepad2.back){
                sleep(200);
            }


            if (gamepad2.right_bumper) {
                sleep(300);
            }

            if (gamepad2.left_bumper) {
                sleep(300);
            }



            telemetry.addLine("Current Pose");
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
            telemetry.addData("slider position",robot.slider.getCurrentPosition());
            telemetry.addData("srm", robot.arm.getCurrentPosition());
            telemetry.addData("collector", robot.collector.getPosition());
            telemetry.addData("intake: ", robot.intake.getDirection());

            telemetry.update();
        }
    }
}
