package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name = "Autonomous StarTech", group = "00-Autonomous", preselectTeleOp = "StarTech")
public class AutonomusBlueLeft extends LinearOpMode {
    public static String TEAM_NAME = "StarTech";
    public static int TEAM_NUMBER = 18338;

    HardwareBox robot = new HardwareBox();

    @Override
    public void runOpMode() throws InterruptedException {

        //Activate Camera Vision that uses TensorFlow for pixel detection

        robot.init(hardwareMap);
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

        moveToChambers = new Pose2d(34,-8,0);

        //identified Spike Mark Location
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        moveBack = new Pose2d(20, 23, Math.toRadians(90));
        pickUpSemple1 = new Pose2d(38, 33, Math.toRadians(90));
        pickUpSemple2 = new Pose2d(38, 15, Math.toRadians(90));
        pickUpSemple3 = new Pose2d(38, 36, Math.toRadians(90));
        moveToBaskets1 = new Pose2d(16, 30, Math.toRadians(135));


        waitSecondsBeforeDrop = 0.5; //TODO: Adjust time to wait for alliance partner to move from board

        //parking left side
        parkPose = new Pose2d(60, 12, Math.toRadians(90));

        robot.openArm();
        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToChambers.position, moveToChambers.heading)
                        .build());

        robot.safeWaitSeconds(1);


        //TODO : Code to put specimen to high chambers

        //Move robot to pick up yellow sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBack.position, moveBack.heading)
                        .build());

        robot.closeArm();
        robot.safeWaitSeconds(0.5);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickUpSemple2.position, pickUpSemple2.heading)
                        .build());

        robot.safeWaitSeconds(waitSecondsBeforeDrop);

        //Move robot to basket and to dropYellow sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(moveToBaskets1,0)
                        .build());
        robot.safeWaitSeconds(waitSecondsBeforeDrop);

        robot.highBasketUp();
        robot.safeWaitSeconds(2);
        robot.closeArm();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickUpSemple1.position, pickUpSemple1.heading)
                        .build());

        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(moveToBaskets1,0)
                        .build());

        robot.highBasketUp();
        robot.safeWaitSeconds(2);
        robot.closeArm();

        robot.safeWaitSeconds(waitSecondsBeforeDrop);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickUpSemple3.position, pickUpSemple3.heading)
                        .build());

        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(moveToBaskets1,0)
                        .build());

        //TODO : Code to drop Semple on Basket
        robot.highBasketUp();
        robot.safeWaitSeconds(2);
        robot.closeArm();

        robot.safeWaitSeconds(0.5);

        //TODO : Code to drop Pixel on Backdrop

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());

    }

}
