package org.firstinspires.ftc.teamcode.Autos;
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
import org.firstinspires.ftc.teamcode.NewRobot.Lift;
import org.firstinspires.ftc.teamcode.NewRobot.AutoOptimize;

@Config
@Autonomous
public class IURISpiral extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        PIDF pidf = new PIDF(hardwareMap, this);
        Lift lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);
        AutoOptimize autoOptimize = new AutoOptimize(hardwareMap, this);


       Action first = drive.actionBuilder(initialPose).
               strafeToConstantHeading(new Vector2d(123, 0)).
               build();
        Action second = drive.actionBuilder(new Pose2d(123, 0, 0)).
                strafeToConstantHeading(new Vector2d(123, -115)).
                build();
        Action third = drive.actionBuilder(new Pose2d(123, -115, 0)).
                strafeToConstantHeading(new Vector2d(10, -115)).
                build();
        Action fourth = drive.actionBuilder(new Pose2d(0, -115, 0)).
                strafeToConstantHeading(new Vector2d(10, -20)).
                build();
        Action fifth = drive.actionBuilder(new Pose2d(10, -20, 0)).
                strafeToConstantHeading(new Vector2d(100, -20)).
                build();
        Action sixth = drive.actionBuilder(new Pose2d(100, -20, 0)).
                strafeToConstantHeading(new Vector2d(100, -90)).
                build();
        Action seventh = drive.actionBuilder(new Pose2d(100, -90, 0)).
                strafeToConstantHeading(new Vector2d(25, -90)).
                build();
        Action eighth = drive.actionBuilder(new Pose2d(25, -90, 0)).
                strafeToConstantHeading(new Vector2d(25, -40)).
                build();
        Action ninth = drive.actionBuilder(new Pose2d(25, -40, 0)).
                strafeToConstantHeading(new Vector2d(75, -40)).
                build();
        Action tenth = drive.actionBuilder(new Pose2d(75, -40, 0)).
                strafeToConstantHeading(new Vector2d(75, -60)).
                build();




        Actions.runBlocking(pidf.initPositions());

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        autoOptimize.speedCollect4()
                       /* first,
                        second,
                        third,
                        fourth,
                        fifth,
                        new ParallelAction(sixth, pidf.retractCollection()),
                        seventh,
                        new ParallelAction(eighth, pidf.retractCollection()),
                        ninth,
                        new ParallelAction(tenth, pidf.retractCollection()),
                        pidf.dropSample()

                        */
                ));
    }
}
