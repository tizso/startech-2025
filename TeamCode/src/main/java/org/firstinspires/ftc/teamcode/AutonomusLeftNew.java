package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous Left New", group = "00-Autonomous", preselectTeleOp = "StarTech")
public class AutonomusLeftNew extends LinearOpMode {
    public static String TEAM_NAME = "StarTech";
    public static int TEAM_NUMBER = 18338;

    HardwareBox robot = new HardwareBox();
    @Override
    public void runOpMode() throws InterruptedException {

        //Activate Camera Vision that uses TensorFlow for pixel detection

        robot.init(hardwareMap);

        robot.arm.setPosition(0.9);

        sleep(200);

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
        Pose2d moveToBaskets0 = new Pose2d(0,0,0);
        //Pose2d moveBack = new Pose2d(0, 0, 0);
        Pose2d pickUpSemple1 = new Pose2d(0, 0, 0);
        Pose2d pickUpSemple2 = new Pose2d(0, 0, 0);
        Pose2d moveToBaskets1 = new Pose2d(0,0,0);
        Pose2d moveToBaskets2 = new Pose2d(0,0,0);
        Pose2d pickUpSemple3 = new Pose2d(0, 0, 0);
        Pose2d moveToBaskets3 = new Pose2d(0,0,0);
        Pose2d parkPose = new Pose2d(0,0, 0);


        //identified Spike Mark Location
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);
        //moveToChambers = new Pose2d(32.3,-8,0);
        moveToBaskets0 = new Pose2d(32.5,-8,0);
        //moveBack = new Pose2d(20, 20, Math.toRadians(90));
        pickUpSemple1 = new Pose2d(37.7, 24, Math.toRadians(90));
        pickUpSemple2 = new Pose2d(36.0, 34, Math.toRadians(90));
        pickUpSemple3 = new Pose2d(35.0, 43, Math.toRadians(90));
        moveToBaskets1 = new Pose2d(8.3, 34.7, Math.toRadians(150));
        moveToBaskets2 = new Pose2d(8.5, 33.5, Math.toRadians(145));


        double waitSecondsBeforeDrop = 0.5; //TODO: Adjust time to wait for alliance partner to move from board

        //TODO parking left side
        parkPose = new Pose2d(60, 12, Math.toRadians(90));


        //TODO Move robot to drop preload sample in basket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToBaskets0.position, moveToBaskets0.heading)
                        .build());

        //TODO Put preload semple in basket
        robot.putInBasket();

        robot.arm.setPosition(0.0);
        robot.rotate.setPosition(1.0);

        //TODO Move robot to pick up fist sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickUpSemple1.position, pickUpSemple1.heading)
                        .build());

        //TODO Pick up first sample
        robot.claw.setPosition(0.9);
        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.arm.setPosition(0.9);

        //TODO Move robot to put first sample in basket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToBaskets1.position, moveToBaskets1.heading)
                        .build());

        //TODO Put first sample in basket
        robot.putInBasket();

        robot.arm.setPosition(0.0);
        robot.rotate.setPosition(1.0);

        //TODO Move robot to pick up second sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickUpSemple2.position, pickUpSemple2.heading)
                        .build());

        //TODO Pick up second sample
        robot.claw.setPosition(0.9);
        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.arm.setPosition(0.9);

        //TODO Move robot to put second sample in basket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToBaskets1.position, moveToBaskets1.heading)
                        .build());

        //TODO Put second sample in basket
        robot.putInBasket();

        robot.rotate.setPosition(1.0);
        robot.arm.setPosition(0.0);
        robot.claw.setPosition(0.3);

        //TODO Move robot to pick up third sample
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pickUpSemple3.position, pickUpSemple3.heading)
                        .build());

        //TODO Pick up third sample
        robot.claw.setPosition(0.9);
        robot.safeWaitSeconds(waitSecondsBeforeDrop);

        //TODO Move robot to put third sample in basket
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToBaskets2.position, moveToBaskets2.heading)
                        .build());

        //TODO Put third sample in basket
        robot.arm.setPosition(0.9);
        robot.safeWaitSeconds(0.5);
        robot.moveSliders(2770,0.9);
        robot.safeWaitSeconds(2.5);
        robot.rotate.setPosition(0.1);
        robot.arm.setPosition(0.65);
        robot.safeWaitSeconds(0.2);
        robot.claw.setPosition(0.1);
        robot.safeWaitSeconds(0.5);
        robot.arm.setPosition(0.9);
        robot.safeWaitSeconds(0.2);
        robot.moveSliders(0,0.9);
        robot.safeWaitSeconds(2.0);



    }
}
