package org.firstinspires.ftc.teamcode.NewRobot;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.OldStuff.PIDF;


@Config
@Autonomous(name = "RedSpecimen", group = "Autonomous")
public class RedSpecimen extends LinearOpMode {


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);


        Action preload = drive.actionBuilder(initialPose).
                afterDisp(4, lift.liftMid()).
                strafeToLinearHeading(new Vector2d(0, 31), Math.toRadians(-90)).
                build();
        Action toCollect = drive.actionBuilder(new Pose2d(0, 30, Math.toRadians(-90))).
                afterDisp(14, pidf.specCollect()).
                strafeToLinearHeading(new Vector2d(20, 22), Math.toRadians(40)).
                build();
        Action collect1 = drive.actionBuilder(new Pose2d(20, 22, Math.toRadians(40))).
                strafeToLinearHeading(new Vector2d(24, 20), Math.toRadians(-35)).
                build();
        Action collect2 = drive.actionBuilder(new Pose2d(24, 20, Math.toRadians(-35))).
                strafeToLinearHeading(new Vector2d(31, 22), Math.toRadians(38)).
                build();
        Action spit2 = drive.actionBuilder(new Pose2d(23, 20, Math.toRadians(30))).
                strafeToLinearHeading(new Vector2d(24, 20), Math.toRadians(-45)).
                build();
        Action collect3 = drive.actionBuilder(new Pose2d(24, 20, Math.toRadians(-35))).
                strafeToLinearHeading(new Vector2d(38, 22), Math.toRadians(35)).
                build();
        Action toPickup = drive.actionBuilder(new Pose2d(36, 22, Math.toRadians(35))).
                strafeToLinearHeading(new Vector2d(40, 5), Math.toRadians(90)).
                build();

        Action toPickup2 = drive.actionBuilder(new Pose2d(-2, 30.3, Math.toRadians(-90))).
                afterDisp(5, lift.liftDown()).
                strafeToLinearHeading(new Vector2d(40, 7), Math.toRadians(90)).
                build();
        Action toPickup3 = drive.actionBuilder(new Pose2d(-6, 30.3, Math.toRadians(-90))).
                afterDisp(5, lift.liftDown()).
                strafeToLinearHeading(new Vector2d(40, 7), Math.toRadians(90)).
                build();
        Action toPickup4 = drive.actionBuilder(new Pose2d(-8, 30.3, Math.toRadians(-90))).
                afterDisp(5, lift.liftDown()).
                strafeToLinearHeading(new Vector2d(40, 7), Math.toRadians(90)).
                build();
        Action pickup = drive.actionBuilder(new Pose2d(40, 7, Math.toRadians(90))).
                strafeToLinearHeading(new Vector2d(40, 0.5), Math.toRadians(90)).
                build();
        Action pickup2 = drive.actionBuilder(new Pose2d(40, 7, Math.toRadians(90))).
                strafeToLinearHeading(new Vector2d(40, 0.5), Math.toRadians(90)).
                build();
        Action pickup3 = drive.actionBuilder(new Pose2d(40, 7, Math.toRadians(90))).
                strafeToLinearHeading(new Vector2d(40, 0.5), Math.toRadians(90)).
                build();
        Action pickup4 = drive.actionBuilder(new Pose2d(40, 7, Math.toRadians(90))).
                strafeToLinearHeading(new Vector2d(40, 0.5), Math.toRadians(90)).
                build();
        Action deliver2 = drive.actionBuilder(new Pose2d(40, 0, Math.toRadians(90))).
                afterDisp(20, lift.liftMid()).
                strafeToSplineHeading(new Vector2d(2, 32.5), Math.toRadians(-90)).
                build();
        Action deliver3 = drive.actionBuilder(new Pose2d(40, 0, Math.toRadians(90))).
                afterDisp(20, lift.liftMid()).
                strafeToSplineHeading(new Vector2d(-4, 33), Math.toRadians(-90)).
                build();
        Action deliver4 = drive.actionBuilder(new Pose2d(40, 0, Math.toRadians(90))).
                afterDisp(20, lift.liftMid()).
                strafeToSplineHeading(new Vector2d(-6, 33), Math.toRadians(-90)).
                build();
        Action deliver5 = drive.actionBuilder(new Pose2d(40, 0, Math.toRadians(90))).
                afterDisp(20, lift.liftMid()).
                strafeToSplineHeading(new Vector2d(-8, 32.5), Math.toRadians(-90)).
                build();
        Action park = drive.actionBuilder(new Pose2d(-6, 30.3, Math.toRadians(-60))).
                splineTo(new Vector2d(40, 20), Math.toRadians(-20)).
                build();
        Action end = drive.actionBuilder(new Pose2d(35, 20, Math.toRadians(90))).
                strafeTo(new Vector2d(44, 3)).
                build();


        Actions.runBlocking(pidf.initPositions());
        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(preload, pidf.retractCollection()),
                        lift.specDeliver(),
                        toCollect,
                        new ParallelAction(collect1, lift.specDown()),
                        pidf.spitOut(),
                        new ParallelAction(collect2, pidf.extendCollection()),
                        pidf.specCollect(),
                        spit2,
                        pidf.spitOut(),
                        new ParallelAction(collect3, pidf.extendCollection()),
                        pidf.collectRun(),
                        //Going to collect 1st from wall
                        new ParallelAction(toPickup,lift.pickup(), pidf.retractCollection()),

                        new ParallelAction(pickup),
                        lift.liftPickup(),
                        new ParallelAction(deliver2),
                        lift.specDeliver(),
                        //Going to collect 2nd from wall
                        new ParallelAction(toPickup2, pidf.retractCollection(), lift.specDown()),

                        pickup2,
                        lift.liftPickup(),
                        new ParallelAction(deliver3),
                        lift.specDeliver(),
                        new ParallelAction(toPickup3, pidf.retractCollection(), lift.specDown()),

                        pickup3,
                        lift.liftPickup(),
                        new ParallelAction(deliver4),
                        lift.specDeliver(),
                        new ParallelAction(toPickup4, pidf.retractCollection(), lift.specDown()),

                        pickup4,
                        lift.liftPickup(),
                        new ParallelAction(deliver5),
                        lift.specDeliver(),
                        new ParallelAction(park, pidf.collectRun(), lift.specDown()),
                        new ParallelAction(end, pidf.retractCollection())

//
//3UHJX3

                ));
    }
}