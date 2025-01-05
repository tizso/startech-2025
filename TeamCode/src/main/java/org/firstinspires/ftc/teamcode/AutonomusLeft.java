package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "Autonomous Left", group = "00-Autonomous", preselectTeleOp = "StarTech")
public class AutonomusLeft extends LinearOpMode {
    public static String TEAM_NAME = "StarTech";
    public static int TEAM_NUMBER = 18338;

    HardwareBox robot = new HardwareBox();

    @Override
    public void runOpMode() throws InterruptedException {

        //Activate Camera Vision that uses TensorFlow for pixel detection

        robot.init(hardwareMap);

        robot.arm.setPosition(0.9);
       /* robot.slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.arm.setDirection(DcMotorEx.Direction.FORWARD);
        robot.arm.setTargetPosition(0);
        robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.arm.setPower(0.9);*/
        sleep(200);
        /*robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.armMotor.setTargetPosition(0);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.liftMotor.setTargetPosition(0);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        // Wait for the DS start button to be touched.
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {

            telemetry.addData("Autonomus for ", TEAM_NAME);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }

    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d moveToChambers = new Pose2d(0,0,0);
        Pose2d moveBack = new Pose2d(0, 0, 0);
        Pose2d pickUpSemple1 = new Pose2d(0, 0, 0);
        Pose2d pickUpSemple2 = new Pose2d(0, 0, 0);
        Pose2d moveToBaskets1 = new Pose2d(0,0,0);
        Pose2d moveToBaskets2 = new Pose2d(0,0,0);
        Pose2d pickUpSemple3 = new Pose2d(0, 0, 0);
        Pose2d moveToBaskets3 = new Pose2d(0,0,0);
        Pose2d parkPose = new Pose2d(0,0, 0);

        double waitSecondsBeforeDrop = 0;



        //identified Spike Mark Location
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        moveToChambers = new Pose2d(24,-8,0);
        moveBack = new Pose2d(20, 20, Math.toRadians(90));
        pickUpSemple1 = new Pose2d(38.2, 24.5, Math.toRadians(90));
        pickUpSemple2 = new Pose2d(37, 34, Math.toRadians(90));
        pickUpSemple3 = new Pose2d(36, 44.5, Math.toRadians(90));
        moveToBaskets1 = new Pose2d(11.0, 37.0, Math.toRadians(150));
        moveToBaskets2 = new Pose2d(11.5, 36.5, Math.toRadians(150));


        waitSecondsBeforeDrop = 0.4; //TODO: Adjust time to wait for alliance partner to move from board

        //parking left side
        parkPose = new Pose2d(60, 12, Math.toRadians(90));

        robot.highChamber(1600);
        //robot.highChamber(1670, 1590);
        robot.moveClaw(0.9);
        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToChambers.position, moveToChambers.heading)
                        .build());

        robot.putSpec();

        //TODO : Code to put specimen to high chambers

        //Move robot to pick up yellow sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBack.position, moveBack.heading)
                        .build());


        robot.moveSliders(0,0.9);
        robot.sliderServo.setPosition(0.5);
        robot.rotate.setPosition(1.0);
        robot.arm.setPosition(0.0);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickUpSemple1.position, pickUpSemple1.heading)
                        .build());


        robot.claw.setPosition(0.9);
        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.arm.setPosition(0.90);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToBaskets1.position, moveToBaskets1.heading)
                        .build());


        robot.moveSliders(3850,0.9);
        robot.safeWaitSeconds(3);
        robot.rotate.setPosition(0.1);
        robot.arm.setPosition(0.80);
        robot.claw.setPosition(0.1);
        robot.moveSliders(0,0.9);
        robot.safeWaitSeconds(1.5);

        robot.arm.setPosition(0.0);
        robot.rotate.setPosition(1);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickUpSemple2.position, pickUpSemple2.heading)
                        .build());

        robot.claw.setPosition(0.9);
        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.arm.setPosition(0.90);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToBaskets1.position, moveToBaskets1.heading)
                        .build());


        robot.moveSliders(3850,0.9);
        robot.safeWaitSeconds(3);
        robot.rotate.setPosition(0.1);
        robot.arm.setPosition(0.80);
        robot.claw.setPosition(0.1);
        robot.moveSliders(0,0.9);
        robot.safeWaitSeconds(1.5);

        robot.rotate.setPosition(1);
        robot.claw.setPosition(0.3);
        robot.arm.setPosition(0.0);


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickUpSemple3.position, pickUpSemple3.heading)
                        .build());

        robot.claw.setPosition(0.9);
        robot.safeWaitSeconds(waitSecondsBeforeDrop);


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToBaskets2.position, moveToBaskets2.heading)
                        .build());

        robot.arm.setPosition(0.90);
        robot.moveSliders(3850,0.9);
        robot.safeWaitSeconds(3);
        robot.rotate.setPosition(0.1);
        robot.arm.setPosition(0.8);
        robot.claw.setPosition(0.1);
        robot.safeWaitSeconds(0.1);
        robot.moveSliders(0,0.9);
        robot.safeWaitSeconds(2);

    }

}
