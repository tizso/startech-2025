package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Autonomous Right", group = "00-Autonomous", preselectTeleOp = "StarTech")
public class AutonomusRight extends LinearOpMode {
    public static String TEAM_NAME = "StarTech";
    public static int TEAM_NUMBER = 18338;

    HardwareBox robot = new HardwareBox();

    double POS_ZERO = 0.0;
    double CLAW_OPEN = 0.9;
    double CLAW_CLOSE = 0.9;
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
        robot.arm.setPosition(0.9);

        while (!isStopRequested() && !opModeIsActive()) {

            telemetry.addData("Autonomus for ", TEAM_NAME);
            telemetry.addData("Slider", robot.slider.getCurrentPosition());
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            telemetry.addData("Slider", robot.slider.getCurrentPosition());
            telemetry.update();
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
        Pose2d smp31 = new Pose2d(0,0,0);
        Pose2d smp3 = new Pose2d(0,0,0);
        Pose2d moveToChambers1 = new Pose2d(0,0,0);
        Pose2d moveToChambers2 = new Pose2d(0,0,0);
        Pose2d parkPose = new Pose2d(0,0,0);

        double waitSecondsBeforeDrop = 0.3;

        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        moveToChambers = new Pose2d(24,10, Math.toRadians(3));
        moveToChambers1 = new Pose2d(24.3,16, Math.toRadians(0));
        moveToChambers2 = new Pose2d(28,18, Math.toRadians(0));
        moveRight = new Pose2d(30,-20, Math.toRadians(180));
        moveBack = new Pose2d(55, -25, Math.toRadians(185));
        backSmp1 = new Pose2d(58,-30, Math.toRadians(185));
        pushSmp1 = new Pose2d(12,-33, Math.toRadians(185));
        moveBack2 = new Pose2d(58,-31, Math.toRadians(185));
        backSmp2 = new Pose2d(58,-41, Math.toRadians(185));
        pushSmp2 = new Pose2d(13.5, -47, Math.toRadians(180));
        smp31 = new Pose2d(11.8, -24, Math.toRadians(180));
        smp3 = new Pose2d(8.8, -24, Math.toRadians(180));
        parkPose = new Pose2d(4, -26, Math.toRadians(180));


        robot.highChamber(1600);
        //robot.highChamber(1670, 1590);
        robot.moveClaw(0.9);
        //Move robot to chamber
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToChambers.position, moveToChambers.heading)
                        .build());

        //Put first spec
        robot.putSpec();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveRight.position, moveRight.heading)
                        .build());
        robot.moveSliders(0,0.9);

        //Move robot to push blue or red sample1 and samp2 to HP
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBack.position, moveBack.heading)
                        .strafeToLinearHeading(backSmp1.position, backSmp1.heading)
                        .strafeToLinearHeading(pushSmp1.position, pushSmp1.heading)
                        .strafeToLinearHeading(moveBack2.position, moveBack2.heading)
                        .strafeToLinearHeading(backSmp2.position, backSmp2.heading)
                        .build());

        if(robot.slider2.getCurrentPosition()<50){
            robot.slider2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }


        robot.moveArm(ARM_WALL);
        //robot.moveSlidersAuto(230, 220,0.9);
        robot.moveSliders(130 ,0.9);

        //go and pick up second spec
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(pushSmp2.position, pushSmp2.heading)
                        .build());

        robot.safeWaitSeconds(0.3);
        robot.moveClaw(CLAW_CLOSE);
        robot.safeWaitSeconds(waitSecondsBeforeDrop);
        robot.moveArm(ARM_CHAMB);
        robot.moveSliders(1550, 0.9);
        //robot.moveSlidersAuto(1450, 1392, 0.9);

        //go to chamber whit spec2
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToChambers.position, moveToChambers.heading)
                        .strafeToLinearHeading(moveToChambers1.position, moveToChambers1.heading)
                        .build());

        robot.putSpec();

        robot.moveArm(ARM_WALL);
        robot.moveSliders(230, 0.9);

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)

                        .strafeToLinearHeading(smp31.position, smp31.heading)
                        .strafeToLinearHeading(smp3.position, smp3.heading)
                        .build());

        robot.moveClaw(CLAW_CLOSE);
        robot.safeWaitSeconds(0.5);
        robot.moveArm(ARM_CHAMB);
        robot.moveSliders(1550, 0.9);

        //robot.moveSlidersAuto(1500, 1430,0.9);
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveToChambers.position, moveToChambers.heading)
                        .strafeToLinearHeading(moveToChambers2.position, moveToChambers2.heading)
                        .build());

        robot.putSpec();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        .build());
    }


}
