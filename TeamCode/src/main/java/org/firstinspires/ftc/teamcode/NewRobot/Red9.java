package org.firstinspires.ftc.teamcode.NewRobot;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;

@Config
@Autonomous(name = "Red 9", group = "Autonomous")
public class Red9 extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);
        AutoOptimize autoOptimize = new AutoOptimize(hardwareMap, this);


        Action toDeliver = drive.actionBuilder(initialPose).
                afterDisp(5, autoOptimize.liftCollect()).
                splineToLinearHeading(new Pose2d(10,22,Math.toRadians(-19)), Math.toRadians(0)).
                build();
        Action score2 = drive.actionBuilder(new Pose2d(12, 23.5, Math.toRadians(-27))).
                turn(Math.toRadians(27)).
                build();
        Action score3 = drive.actionBuilder(new Pose2d(10, 19, -45)).
                strafeToLinearHeading(new Vector2d(8, 18), Math.toRadians(-20)).
                build();
        Action collect3 = drive.actionBuilder(initialPose).
                strafeToLinearHeading(new Vector2d(15, 17), Math.toRadians(47)).
                build();
        Action score4 = drive.actionBuilder(new Pose2d(10, 19, -45)).
                strafeToLinearHeading(new Vector2d(6.5, 19), Math.toRadians(-35)).
                build();
        Action num7 = drive.actionBuilder(new Pose2d(5, 15, -45)).
                strafeToLinearHeading(new Vector2d(2, -13), -90).
                build();
        Action score7 = drive.actionBuilder(new Pose2d(9,-8,-45)).
                strafeToConstantHeading(new Vector2d(9, 14)).
                build();
        Action sub = drive.actionBuilder(new Pose2d(10, 17, Math.toRadians(-20))).
                afterDisp(3, lift.liftDown()).
                //afterDisp(48, pidf.sweeperOut()).
                afterDisp(32, autoOptimize.speedCollect3()).
                splineTo(new Vector2d(52, -15.5), Math.toRadians(-93)).
                build();
        Action afterSub = drive.actionBuilder(new Pose2d(52,-11, -45)).
                afterDisp(6, autoOptimize.speedCollect2()).
                strafeToConstantHeading(new Vector2d(8, 18)).
                build();
        Action collect6 = drive.actionBuilder(new Pose2d(12, 17, Math.toRadians(-10))).
                setReversed(false).
                afterDisp(4, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(44, autoOptimize.speedCollect3()).
                splineTo(new Vector2d(48, -15), Math.toRadians(-100)).
                build();
        Action jk = drive.actionBuilder(new Pose2d(52,-13,-45)).
                setReversed(true).
                afterDisp(5, autoOptimize.speedCollect2()).
                strafeToConstantHeading(new Vector2d(8.7, 18)).
                build();
        //Actually collecting 8
        Action heading = drive.actionBuilder(new Pose2d(10, 18, Math.toRadians(-30))).
                setReversed(false).
                afterDisp(4, lift.liftDown()).
                afterDisp(52, pidf.sweeperOut()).
                afterDisp(50, autoOptimize.speedCollect3()).
                splineTo(new Vector2d(53.5, -16), Math.toRadians(-90)).
                build();
        //Actually collecting 7
        Action collect8 = drive.actionBuilder(new Pose2d(10, 18, Math.toRadians(-22))).
                setReversed(false).
                afterDisp(4, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(41, autoOptimize.speedCollect3()).
                splineTo(new Vector2d(51, -15.5), Math.toRadians(-86)).
                build();
        //Collecting 9
        Action collectLast = drive.actionBuilder(new Pose2d(10, 18, Math.toRadians(-24))).
                setReversed(false).
                afterDisp(4, lift.liftDown()).
                afterDisp(55, pidf.sweeperOut()).
                afterDisp(45, autoOptimize.speedCollect3()).
                splineTo(new Vector2d(47, -15), Math.toRadians(-108)).
                build();
        Action score8 = drive.actionBuilder(new Pose2d(50, -13,-45)).
                setReversed(true).
                afterDisp(7, autoOptimize.speedCollect2()).
                strafeToConstantHeading(new Vector2d(8, 16)).
                build();
        Action score9 = drive.actionBuilder(new Pose2d(50, -13,-45)).
                setReversed(true).
                afterDisp(6, autoOptimize.speedCollect2()).
                strafeToConstantHeading(new Vector2d(8, 16)).
                build();
        Action scoreLast = drive.actionBuilder(new Pose2d(50, -13,-45)).
                setReversed(true).
                afterDisp(5.5, autoOptimize.speedCollect2()).
                strafeToConstantHeading(new Vector2d(8, 17)).
                build();
        Action end = drive.actionBuilder(new Pose2d(6.5, 14.3, Math.toRadians(-30))).
                afterDisp(1, lift.liftDown()).
                splineTo(new Vector2d(30, 0), Math.toRadians(0)).
                build();
        Action score10 = drive.actionBuilder(new Pose2d(10, -10,-45)).
                strafeToConstantHeading(new Vector2d(6.5, 16.5)).
                build();
        Action collect10 = drive.actionBuilder(new Pose2d(10, 18, Math.toRadians(-90))).
                strafeToConstantHeading(new Vector2d(-1, -10)).
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
                        new ParallelAction(collectLast, pidf.sweeperIn()),
                        new ParallelAction(scoreLast),
                        new ParallelAction(collect10, pidf.sweeperIn(), autoOptimize.speedCollect(), lift.liftDown()),
                        new ParallelAction(score10, autoOptimize.speedCollect2()),
                        new ParallelAction(end, pidf.retractCollection(), lift.liftDown())

                ));
    }
}
