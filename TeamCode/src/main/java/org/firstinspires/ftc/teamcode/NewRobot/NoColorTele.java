package org.firstinspires.ftc.teamcode.NewRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.OldStuff.PIDF;


@TeleOp(name = "NoColor")
public class NoColorTele extends OpMode {

    PIDF pidf;
    Drivetrain drivetrain;

    Lift lift;



    @Override
    public void init() {
        lift = new Lift(hardwareMap, this, 145.1, 1, 1.15);
        pidf = new PIDF(hardwareMap, this);
        drivetrain = new Drivetrain(hardwareMap, this, 384.5, 1, 4);


    }
    @Override
    public void loop() {
        drivetrain.UpdateDriveTrain();
        lift.soloControls();
        pidf.IURITele();
        //pidf.testSensors();

    }


    @Override
    public void stop() {
    }

}
