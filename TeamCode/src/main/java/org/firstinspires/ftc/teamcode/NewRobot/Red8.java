package org.firstinspires.ftc.teamcode.NewRobot;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;

@Config
@Autonomous(name = "Red 10", group = "Autonomous")
public class Red8 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);
        AutoOptimize autoOptimize = new AutoOptimize(hardwareMap, this);


        Action toDeliver = drive.actionBuilder(initialPose).
                afterDisp(5, autoOptimize.liftCollect()).
                splineToLinearHeading(new Pose2d(12,24,Math.toRadians(-24)), Math.toRadians(0)).
                build();
        Action score2 = drive.actionBuilder(new Pose2d(12, 23.5, Math.toRadians(-27))).
                turn(Math.toRadians(27)).
                build();
        Action score3 = drive.actionBuilder(new Pose2d(10, 19, -45)).
                strafeToLinearHeading(new Vector2d(8, 18), Math.toRadians(-20)).
                build();
        Action collect3 = drive.actionBuilder(initialPose).
                strafeToLinearHeading(new Vector2d(16.8, 18), Math.toRadians(40)).
                build();
        Action score4 = drive.actionBuilder(new Pose2d(10, 19, -45)).
                strafeToLinearHeading(new Vector2d(6.5, 19), Math.toRadians(-35)).
                build();
        Action sub = drive.actionBuilder(new Pose2d(10, 17, Math.toRadians(-27))).
                afterDisp(3, lift.liftDown()).
                //afterDisp(48, pidf.sweeperOut()).
                        afterDisp(46, autoOptimize.speedCollect1()).
                splineTo(new Vector2d(48, -15.5), Math.toRadians(-93)).
                build();
        Action afterSub = drive.actionBuilder(new Pose2d(50,-13, Math.toRadians(-90))).
                setReversed(true).
                afterDisp(5.5, autoOptimize.speedCollect2()).
                splineTo(new Vector2d(11, 16.5), Math.toRadians(140), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.8)).
                build();
        Action collect6 = drive.actionBuilder(new Pose2d(9, 16.5, Math.toRadians(-38))).
                afterDisp(3, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(40, autoOptimize.speedCollect()).
                splineTo(new Vector2d(50, -14), Math.toRadians(-100)).
                build();
        Action jk = drive.actionBuilder(new Pose2d(50,-13,Math.toRadians(-90))).
                setReversed(true).
                afterDisp(5.5, autoOptimize.speedCollect2()).
                splineTo(new Vector2d(11.5, 17.5), Math.toRadians(138), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.8)).
                build();
        Action heading = drive.actionBuilder(new Pose2d(9, 16.5, Math.toRadians(-38))).
                afterDisp(3, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(46.5, autoOptimize.speedCollect()).
                splineTo(new Vector2d(56, -14.5), Math.toRadians(-90)).
                build();
        Action collect8 = drive.actionBuilder(new Pose2d(9, 16.5, Math.toRadians(-38))).
                afterDisp(3, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(43, autoOptimize.speedCollect()).
                splineTo(new Vector2d(54, -14.5), Math.toRadians(-80)).
                build();
        Action score8 = drive.actionBuilder(new Pose2d(50, -13,Math.toRadians(-90))).
                setReversed(true).
                afterDisp(6, autoOptimize.speedCollect2()).
                splineTo(new Vector2d(12, 15.5), Math.toRadians(138), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.8)).
                build();
        Action score9 = drive.actionBuilder(new Pose2d(50, -13,Math.toRadians(-90))).
                setReversed(true).
                afterDisp(6, autoOptimize.speedCollect2()).
                splineTo(new Vector2d(12, 15), Math.toRadians(138), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.8)).
                build();
        Action collect9 = drive.actionBuilder(new Pose2d(10, 18, Math.toRadians(-38))).
                setReversed(false).
                afterDisp(3, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(45, autoOptimize.speedCollect()).
                splineTo(new Vector2d(53, -14.5), Math.toRadians(-90)).
                build();
        Action scoreLast = drive.actionBuilder(new Pose2d(50, -13, Math.toRadians(-90))).
                setReversed(true).
                afterDisp(6, autoOptimize.speedCollect2()).
                splineTo(new Vector2d(9, 15), Math.toRadians(140), new TranslationalVelConstraint(MecanumDrive.PARAMS.maxWheelVel * 1.8)).
                build();
        Action num7 = drive.actionBuilder(new Pose2d(5, 15, -45)).
                strafeToLinearHeading(new Vector2d(2, -13), -90).
                build();
        Action score7 = drive.actionBuilder(new Pose2d(9,-8,-45)).
                strafeToConstantHeading(new Vector2d(9, 14)).
                build();
        Action end = drive.actionBuilder(new Pose2d(6.5, 14.3, Math.toRadians(-30))).
                afterDisp(1, lift.liftDown()).
                splineTo(new Vector2d(25, 0), Math.toRadians(0)).
                build();
        Actions.runBlocking(pidf.initPositions());

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(toDeliver),
                        lift.liftUp(),
                        new ParallelAction(score2, lift.liftDown(), pidf.collectRun()),
                        new ParallelAction(score3, lift.liftUp()),
                        new ParallelAction(collect3, pidf.collectRun(),lift.liftDown()),
                        new ParallelAction(score4, lift.liftUp(), pidf.retractCollection()),
                        new ParallelAction(sub, pidf.sweeperIn()),
                        new ParallelAction(afterSub),
                        new ParallelAction(collect6, pidf.sweeperIn()),
                        new ParallelAction(jk),
                        new ParallelAction(collect8, pidf.sweeperIn()),
                        new ParallelAction(score8),
                        new ParallelAction(heading, pidf.sweeperIn()),
                        new ParallelAction(score9),
                        new ParallelAction(collect9, pidf.sweeperIn(), pidf.retractCollection()),
                        new ParallelAction(scoreLast),
                        new ParallelAction(num7, autoOptimize.speedCollect(), lift.liftDown()),
                        new ParallelAction(score7, autoOptimize.speedCollect2()),
                        new ParallelAction(end, pidf.retractCollection())

                ));
    }
}
