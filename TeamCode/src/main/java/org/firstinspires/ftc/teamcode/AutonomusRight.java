package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Autonomous Right", group = "00-Autonomous", preselectTeleOp = "StarTech")
public class AutonomusRight extends LinearOpMode {
    public static String TEAM_NAME = "StarTech";
    public static int TEAM_NUMBER = 18338;

    HardwareBox robot = new HardwareBox();

    double POS_ZERO = 0.0;
    double CLAW_OPEN = 0.9;
    double CLAW_CLOSE = 0.65;
    double ARM_WALL = 0.5;
    double ARM_CHAMB = 0.9;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
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
        Pose2d moveToChambers = new Pose2d(0,0,0);
        Pose2d moveRight = new Pose2d(0,0,0);
        Pose2d moveBack = new Pose2d(0, 0, 0);
        Pose2d backSmp1 = new Pose2d(0,0,0);
        Pose2d pushSmp1 = new Pose2d(0,0,0);
        Pose2d moveBack2 = new Pose2d(0,0,0);
        Pose2d backSmp2 = new Pose2d(0,0,0);
        Pose2d pushSmp2 = new Pose2d(0,0,0);
        Pose2d smp3 = new Pose2d(0,0,0);
        Pose2d moveToChambers1 = new Pose2d(0,0,0);
        Pose2d moveToChambers2 = new Pose2d(0,0,0);

        Pose2d parkPose = new Pose2d(0,0, 0);

        double waitSecondsBeforeDrop = 0.3;

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        moveToChambers = new Pose2d(24,12,0);
        moveToChambers1 = new Pose2d(21.5,15, Math.toRadians(0));
        moveToChambers2 = new Pose2d(21,14, Math.toRadians(0));
        moveRight = new Pose2d(30,-20, Math.toRadians(180));
        moveBack = new Pose2d(55, -25, Math.toRadians(181));
        backSmp1 = new Pose2d(58,-30, Math.toRadians(180));
        pushSmp1 = new Pose2d(12,-33, Math.toRadians(180));
        moveBack2 = new Pose2d(58,-31, Math.toRadians(181));
        backSmp2 = new Pose2d(58,-40, Math.toRadians(180));
        pushSmp2 = new Pose2d(13.7, -47, Math.toRadians(180));
        smp3 = new Pose2d(10.5, -25, Math.toRadians(190));



        //parking left side
        parkPose = new Pose2d(60, 12, Math.toRadians(90));

        robot.highChamber(1670);
        robot.moveClaw(0.0);
        //Move robot to dropPurlePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToChambers.position, moveToChambers.heading)
                        .build());
        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.moveArm(0.5);
        robot.safeWaitSeconds(0.1);
        robot.moveArm(0.3);
        robot.safeWaitSeconds(0.2);
        robot.moveClaw(CLAW_OPEN);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveRight.position, moveRight.heading)
                        .build());
        robot.moveSliders(10, 0.9);


        //Move robot to push blue or red sample to HP
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBack.position, moveBack.heading)
                        .strafeToLinearHeading(backSmp1.position, backSmp1.heading)
                        .strafeToLinearHeading(pushSmp1.position, pushSmp1.heading)
                        .strafeToLinearHeading(moveBack2.position, moveBack2.heading)
                        .strafeToLinearHeading(backSmp2.position, backSmp2.heading)
                        .build());

        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.moveArm(ARM_WALL);
        robot.moveSliders(250,0.9);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pushSmp2.position, pushSmp2.heading)
                        .build());

        robot.moveClaw(CLAW_CLOSE);
        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.moveArm(ARM_CHAMB);
        robot.moveSliders(1450, 0.9);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToChambers.position, moveToChambers.heading)
                        .strafeToLinearHeading(moveToChambers1.position, moveToChambers1.heading)
                        .build());
        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.moveArm(0.5);
        robot.safeWaitSeconds(0.1);
        robot.moveArm(0.3);
        robot.safeWaitSeconds(0.2);
        robot.moveClaw(CLAW_OPEN);
        robot.moveArm(ARM_WALL);
        robot.moveSliders(250, 0.9);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(smp3.position, smp3.heading)
                        .build());
        robot.moveClaw(CLAW_CLOSE);
        robot.safeWaitSeconds(0.5);
        robot.moveArm(ARM_CHAMB);
        robot.moveSliders(1500, 0.9);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToChambers.position, moveToChambers.heading)
                        .strafeToLinearHeading(moveToChambers2.position, moveToChambers2.heading)
                        .build());
        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.moveArm(0.5);
        robot.safeWaitSeconds(0.1);
        robot.moveArm(0.3);
        robot.safeWaitSeconds(0.2);
        robot.moveClaw(CLAW_OPEN);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(smp3.position, smp3.heading)
                        .build());
        //Move robot to parking
        /*Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());
*/
    }


}
