package org.firstinspires.ftc.teamcode.OldStuff;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDF {


    public enum ExtendState {
        START,
        RETRACT,
        MID,
        EXTEND,
        EXTENDED,
        EJECT,
        TRANSFER,
        SPECIMEN,
        RESET,
        REJECT,
        DELIVER,
        INTAKE,
        SPECIMEN2,

    }

    ExtendState extendState = ExtendState.START;

    PIDController controller;
    public static double p = 0.007, i = 0.000001, d = 0.000008;
    public static int target;


    OpMode theOpMode;
    DcMotorEx extend;
    DcMotorEx collection;
    public Servo rCollection;
    public Servo lCollection;
    public Servo claw;
    public Servo deliveryS;
    public Servo sweep;
    public Servo door;
    public CRServo hang;
    public CRServo hang2;

    int extended = 565;
    int retracted = 10;
    int mid = 300;
    int shortPos = 100;
    double collect = .44;
    double transfer = .35;
    double xHeight = .24;
    double initPos = .12;

    double closed = .627;
    double open = .42;
    double specClosed = .608;
    double backSpec = .71;
    double transferPos = .0;
    double midPos = .53;
    double lowerMid = .15;
    double specPos = .675;

    double in = .4;
    double out = .705;

    DigitalChannel cBeam;
    DigitalChannel dBeam;
    DigitalChannel lSwitch;
    DigitalChannel touch;
    DigitalChannel liftTouch;
    ColorSensor colorSensor;
    double sensorReading;
    ElapsedTime transferTimer = new ElapsedTime();
    ElapsedTime beamTimer = new ElapsedTime();
    ElapsedTime failTimer = new ElapsedTime();

    public PIDF(HardwareMap hardwareMap, OpMode opMode) {
        theOpMode = opMode;
        controller = new PIDController(p, i, d);
        extend = hardwareMap.get(DcMotorEx.class, "extend");
        extend.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        extend.setDirection(DcMotorSimple.Direction.FORWARD);
        lCollection = hardwareMap.get(Servo.class, "lCollection");
        rCollection = hardwareMap.get(Servo.class, "rCollection");
        rCollection.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.get(Servo.class, "claw");
        deliveryS = hardwareMap.get(Servo.class, "delivery");

        hang = hardwareMap.get(CRServo.class, "hang");
        hang2 = hardwareMap.get(CRServo.class, "hang2");
        collection = hardwareMap.get(DcMotorEx.class, "collection");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);

        cBeam = hardwareMap.get(DigitalChannel.class, "beam");
        cBeam.setMode(DigitalChannel.Mode.INPUT);
        dBeam = hardwareMap.get(DigitalChannel.class, "dBeam");
        dBeam.setMode(DigitalChannel.Mode.INPUT);
        lSwitch = hardwareMap.get(DigitalChannel.class, "lSwitch");
        lSwitch.setMode(DigitalChannel.Mode.INPUT);
        sweep = hardwareMap.get(Servo.class, "sweep");
        door = hardwareMap.get(Servo.class, "door");

        liftTouch = hardwareMap.get(DigitalChannel.class, "lTouch");
        liftTouch.setMode(DigitalChannel.Mode.INPUT);


    }

    public void testSensors() {
        if (!cBeam.getState()) {
            theOpMode.telemetry.addData("Collection Beam", "Broken");
        } else {
            theOpMode.telemetry.addData("Collection Beam", "Not Broken");
        }
        if (!dBeam.getState()) {
            theOpMode.telemetry.addData("Delivery Beam", "Broken");
        }
        else {
            theOpMode.telemetry.addData("Delivery Beam", "Not Broken");
        }
        theOpMode.telemetry.update();
    }

    public void tele(boolean isRed) {
//        if (theOpMode.gamepad1.dpad_right) {
//            sweep.setPosition(in);
//        }
        if (theOpMode.gamepad2.right_bumper) {
            sweep.setPosition(in);
        }
        else if (theOpMode.gamepad2.left_bumper) {
            sweep.setPosition(out);
        }
        if (theOpMode.gamepad2.dpad_left) {
            door.setPosition(.4);
        }
        else if (theOpMode.gamepad2.dpad_right) {
            door.setPosition(.15);
        }

        if (theOpMode.gamepad1.left_bumper || theOpMode.gamepad2.a) {
            failTimer.reset();
            claw.setPosition(open);
           // deliveryS.setPosition(backSpec);
            extendState = ExtendState.DELIVER;

        } else if (theOpMode.gamepad1.right_bumper || theOpMode.gamepad2.b) {
            claw.setPosition(closed);
        }
        if (theOpMode.gamepad1.dpad_up) {
            deliveryS.setPosition(backSpec);

        } else if (theOpMode.gamepad1.dpad_down) {
            deliveryS.setPosition(transferPos);
        }

        if (theOpMode.gamepad1.b) {
            extendState = ExtendState.EJECT;
        }
        if (theOpMode.gamepad1.dpad_right) {
            extendState = ExtendState.INTAKE;
        }
        if (theOpMode.gamepad1.ps) {
            extendState = ExtendState.RESET;
        }
        switch (extendState) {
            case DELIVER:
                if (failTimer.seconds() >= .3) {
                    deliveryS.setPosition(midPos);
                    extendState = ExtendState.START;
                }
                break;



                //Fully Retracted in transfer position
            case START:
                //sweep.setPosition(0);
                collection.setPower(0);
                rCollection.setPosition(xHeight);
                lCollection.setPosition(xHeight);

                if (theOpMode.gamepad1.dpad_left) {
                    deliveryS.setPosition(backSpec);
                    claw.setPosition(open);
                    extendState = ExtendState.SPECIMEN;
                }
                // Start extending, turn on collection
                if (theOpMode.gamepad1.x) {
                    target = extended;
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.EXTEND;
                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.95);
                }


                break;
            case RESET:
                target = -600;
                if (!touch.getState()) {
                    deliveryS.setPosition(midPos);
                    extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    target = 0;
                    extendState = ExtendState.START;
                }
                break;
            // Rotate to collecting position

            case SPECIMEN:
                deliveryS.setPosition(backSpec);
                claw.setPosition(open);
                if (!dBeam.getState()) {
                    transferTimer.reset();
                    claw.setPosition(specClosed);
                    extendState = ExtendState.SPECIMEN2;
                }
                break;

            case SPECIMEN2:
                if (transferTimer.seconds() >= .4) {
                    deliveryS.setPosition(transferPos);
                    extendState = ExtendState.START;
                }
                break;

            case EXTEND:
                target = extended;
                if (Math.abs(extend.getCurrentPosition() - extended) < 180 && theOpMode.gamepad1.x) {
                    extendState = ExtendState.EXTENDED;
                    collection.setPower(.95);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                }
                if (!dBeam.getState()) {
                    door.setPosition(.15);
                    collection.setPower(-.9);
                    transferTimer.reset();
                    claw.setPosition(closed);
                    extendState = ExtendState.TRANSFER;
                    theOpMode.telemetry.addData("DBeam", "Broken");
                    theOpMode.telemetry.update();
                }

                break;
            case EXTENDED:
            case MID:

                // If we collect a specimen, retract extension, rotate to transfer position, stop collection
                collection.setPower(.95);
                door.setPosition(.15);
                rCollection.setPosition(collect);
                lCollection.setPosition(collect);

                if (!cBeam.getState()) {
                    deliveryS.setPosition(lowerMid);
                    target = retracted;
                    collection.setPower(.3);
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.RETRACT;
                    theOpMode.telemetry.addData("Beam", "Broken");
                    beamTimer.reset();
                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.95);
                }
                if (theOpMode.gamepad1.x) {
                    target = extended;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.EXTENDED;
                    collection.setPower(.95);

                }
                break;
            case RETRACT:

                if (((double) colorSensor.red() / colorSensor.blue()) >= 1.4 && ((double) colorSensor.red() / colorSensor.alpha() >= 1.3) && !isRed && (Math.abs(extend.getCurrentPosition())  >= 70)) {
                    door.setPosition(.4);
                    collection.setPower(.6);
                    extendState = ExtendState.EXTEND;
                }
                if ((double) colorSensor.blue() / colorSensor.red() >= 1.42 && isRed && (Math.abs(extend.getCurrentPosition())  >= 70)) {
                    door.setPosition(.4);
                    collection.setPower(.6);
                    extendState = ExtendState.EXTEND;
                }


                claw.setPosition(open);
                if (Math.abs(extend.getCurrentPosition() - retracted) < 100) {
                    deliveryS.setPosition(transferPos);
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    collection.setPower(0);
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 70) {
                        door.setPosition(.4);
                    }
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 40 && !liftTouch.getState()) {
                        collection.setPower(.48);
                    }
                    if (extend.getCurrentPosition() - retracted > 40 && beamTimer.seconds() > 2.5) {
                        extend.setPower(-.4);
                    }
                }
                if (!dBeam.getState()) {
                    door.setPosition(.15);
                    collection.setPower(-1);
                    transferTimer.reset();
                    claw.setPosition(closed);
                    extendState = ExtendState.TRANSFER;
                    theOpMode.telemetry.addData("DBeam", "Broken");
                    theOpMode.telemetry.update();
                }

                break;
            case TRANSFER:

                theOpMode.telemetry.addData("transferTimer", transferTimer);
                theOpMode.telemetry.update();
                target = shortPos;
                if (transferTimer.seconds() >= .15) {
                    collection.setPower(0);
                    target = retracted;
                    claw.setPosition(closed);
                    deliveryS.setPosition(backSpec);
                    extendState = ExtendState.START;
                }
                break;
            case EJECT:
                if (theOpMode.gamepad1.b) {
                    collection.setPower(-.45);
                } else {
                    collection.setPower(0);
                    extendState = ExtendState.START;
                }
                break;
            case INTAKE:
                if (theOpMode.gamepad1.dpad_right) {
                    collection.setPower(.45);
                } else {
                    collection.setPower(0);
                    extendState = ExtendState.START;
                }
                break;


            case REJECT:
                target = extended;
                collection.setPower(-.6);
                if (Math.abs(extend.getCurrentPosition() - extended) < 50 && cBeam.getState()) {
                    collection.setPower(.95);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.EXTENDED;
                }
                break;

            default:
                extendState = ExtendState.START;
        }
        if (theOpMode.gamepad1.a && extendState != ExtendState.START) {
            rCollection.setPosition(transfer);
            lCollection.setPosition(transfer);
            collection.setPower(0);
            target = retracted;
            extendState = ExtendState.START;
        }
        controller.setPID(p, i, d);
        int curPos = extend.getCurrentPosition();
        double power = controller.calculate(curPos, target);
        extend.setPower(power);
       // theOpMode.telemetry.addData("pos", curPos);
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.addData("Current State", extendState);
        theOpMode.telemetry.addData("power", power);
        theOpMode.telemetry.addData("Current Pos", curPos);
//        theOpMode.telemetry.addData("delivery position", deliveryS.getPosition());
        if (!dBeam.getState()) {
            theOpMode.telemetry.addData("Delivery Beam", "Broken");
        } else {
            theOpMode.telemetry.addData("Delivery Beam", "Not Broken");
        }
        if (!cBeam.getState()) {
            theOpMode.telemetry.addData("Collection Beam", "Broken");
        } else {
            theOpMode.telemetry.addData("Collection Beam", "Not Broken");
        }
        theOpMode.telemetry.update();
    }


    public void testColor(boolean isRed) {
        if (isRed) {
            sensorReading = colorSensor.red();

        } else {
            sensorReading = colorSensor.blue();
        }

        switch (extendState) {

            //Fully Retracted in transfer position
            case START:
                collection.setPower(0);
                rCollection.setPosition(xHeight);
                lCollection.setPosition(xHeight);

                if (theOpMode.gamepad1.dpad_left) {
                    extendState = ExtendState.SPECIMEN;
                }
                // Start extending, turn on collection

                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.95);
                }


                break;
            case MID:

                // If we collect a specimen, retract extension, rotate to transfer position, stop collection

                if (!cBeam.getState()) {
                    deliveryS.setPosition(transferPos);
                    target = retracted;
                    collection.setPower(0);
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.RETRACT;

                }
                break;
            default:
                extendState = ExtendState.START;
        }
        if (theOpMode.gamepad1.a && extendState != ExtendState.START) {
            rCollection.setPosition(transfer);
            lCollection.setPosition(transfer);
            collection.setPower(0);
            target = retracted;
            extendState = ExtendState.START;
        }
        controller.setPID(p, i, d);
        int curPos = extend.getCurrentPosition();
        double power = controller.calculate(curPos, target);
        extend.setPower(power);

//        theOpMode.telemetry.addData("delivery position", deliveryS.getPosition());
        theOpMode.telemetry.addData("alpha", colorSensor.alpha());
        theOpMode.telemetry.addData("How much red?", colorSensor.red());
        theOpMode.telemetry.addData("How Much Blue?", colorSensor.blue());
        theOpMode.telemetry.addData("IsRed?", isRed);
        theOpMode.telemetry.update();


        //If We are Red
        if ((double) colorSensor.blue() / colorSensor.red() >= 1.5) {
//If too much blue is sensed, we spit it out, otherwise we keep collecting :)
            theOpMode.telemetry.addData("Blue", "");
            theOpMode.telemetry.update();
        }


        //If We are Blue
        if (((double) colorSensor.red() / colorSensor.blue()) >= 1.5 && ((double) colorSensor.red() / colorSensor.alpha() >= 1.25)) {
            //If too much red is sensed AND we aren't sensing a big spike in alpha values, we spit it out
            theOpMode.telemetry.addData("Red", "");
            theOpMode.telemetry.update();
        }


    }


    public class CollectRun implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.15);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);

                    break;
                // Rotate to collecting position
                case EXTEND:
                    target = extended;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    if (Math.abs(extend.getCurrentPosition() - target) < 350) {

                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!cBeam.getState()) {
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();


                    }

                    break;
                case EXTENDED:
                    if (beamTimer.seconds() > 0.85) {
                        target = shortPos;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            target = extended;
                            beamTimer.reset();
                        }
                        if (failTimer.seconds() >= 1.6) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            deliveryS.setPosition(midPos);
                            return false;
                        }
                    }

                    // If we collect a specimen, retract extension, rotate to transfer position, stop collection
                    if (!cBeam.getState()) {
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();


                    }

                    break;
                case RETRACT:
                    door.setPosition(.3);
                    claw.setPosition(open);

                    if (Math.abs(extend.getCurrentPosition() - retracted) < 130) {
                        deliveryS.setPosition(transferPos);
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        door.setPosition(.4);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 45) {
                            collection.setPower(.5);

                            if (beamTimer.seconds() >= 1.5 && cBeam.getState()) {
                                collection.setPower(0);
                                extendState = ExtendState.EXTEND;
                            }
                            if (beamTimer.seconds() >= 1.5 && !cBeam.getState()) {
                                extendState = ExtendState.EJECT;

                            }
                        }



                    }
                    if (!dBeam.getState()) {
                        door.setPosition(.15);
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.9);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }

                    break;
                case EJECT:
                    if (beamTimer.seconds() >= 1.5 && cBeam.getState()) {
                        collection.setPower(0);
                        extendState = ExtendState.EXTEND;
                    }
                    collection.setPower(-.4);
                    if (beamTimer.seconds() >= 1.62) {
                        collection.setPower(.75);
                    }
                    if (beamTimer.seconds() >= 2.2) {
                        extendState = ExtendState.EXTEND;
                    }

                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
            break;
                case TRANSFER:
                    target = shortPos;
                    if (transferTimer.seconds() >= .1) {
                        extend.setPower(0);
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        claw.setPosition(closed);
                        deliveryS.setPosition(midPos);
                        return false;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);
            theOpMode.telemetry.addData("Current Position", curPos);
            theOpMode.telemetry.addData("Target Position", target);
            if (!lSwitch.getState()) {
                theOpMode.telemetry.addData("True", "");
            }
            else {
                theOpMode.telemetry.addData("False", "");
            }
            if (!dBeam.getState()) {
                theOpMode.telemetry.addData("Beam", "Broken");
            } else {
                theOpMode.telemetry.addData("Beam", "Not Broken");
            }
            theOpMode.telemetry.update();
            return true;
        }
    }

    public Action collectRun() {
        return new CollectRun();
    }

    public class InitPositions implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //  switch (extendState) {
            //  case START:
            rCollection.setPosition(initPos);
            lCollection.setPosition(initPos);
            deliveryS.setPosition(midPos);
            claw.setPosition(closed);
            target = -50;
            return false;
            //  break;
                    /*
                case RETRACT:
                    if (!touch.getState()) {
                        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        target = 0;
                        extend.setPower(0);
                        extendState = ExtendState.START;
                    }

                    return false;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            return true;
        }

                     */

        }
    }

    public Action initPositions() {
        return new InitPositions();
    }

    public class ExtendCollection implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                case START:
                    extendState = ExtendState.EXTEND;
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                case EXTEND:
                    target = 270;
                    if (Math.abs(extend.getCurrentPosition() - target) < 45) {
                        extend.setPower(0);
                        extendState = ExtendState.START;
                        return false;
                    }
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power * 1.4);
            theOpMode.telemetry.addData("pos", curPos);
            theOpMode.telemetry.addData("target", target);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("power", power);
            theOpMode.telemetry.update();
            return true;
        }
    }

    public Action extendCollection() {
        return new ExtendCollection();
    }

    public class RetractCollection implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                case START:
                    extendState = ExtendState.EXTEND;
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                case EXTEND:
                    target = 20;
                    if (Math.abs(extend.getCurrentPosition() - target) < 40) {
                        extend.setPower(0);
                        extendState = ExtendState.START;
                        theOpMode.telemetry.addData("Finished", "");
                        theOpMode.telemetry.update();
                        return false;
                    }
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            return true;
        }
    }

    public Action retractCollection() {
        return new RetractCollection();
    }

    public class SubCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.15);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    target = extended;

                    if (Math.abs(extend.getCurrentPosition() - target) < 350) {
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
                    break;
                case EXTENDED:
                    door.setPosition(.15);
                    target = extended;
                    if (beamTimer.seconds() > 0.8) {
                        target = mid;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            target = extended;
                            beamTimer.reset();
                        }
                        if (failTimer.seconds() >= 2.8) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            deliveryS.setPosition(midPos);
                            return false;
                        }
                    }

                    // If we collect a sample, retract extension, rotate to transfer position, stop collection
                    if (!cBeam.getState()) {
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();


                    }
                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
                    break;
                case RETRACT:
                    if ((double) colorSensor.blue() / colorSensor.red() >= 1.4 && (Math.abs(extend.getCurrentPosition())  >= 70)) {
                        beamTimer.reset();
                        collection.setPower(-.5);
                        extendState = ExtendState.REJECT;
                    }

                    claw.setPosition(open);
                    if (beamTimer.seconds() >= 1.2 && cBeam.getState()) {
                        extendState = ExtendState.EXTEND;
                    }
                    if (beamTimer.seconds() >= 1.5 && !cBeam.getState()) {
                        extendState = ExtendState.EJECT;
                    }

                    if (Math.abs(extend.getCurrentPosition() - retracted) < 55) {
                        deliveryS.setPosition(transferPos);
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 21) {
                            door.setPosition(.5);
                            collection.setPower(.5);

                        }


                        if (!dBeam.getState()) {
                            door.setPosition(.15);
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            collection.setPower(-.95);
                            transferTimer.reset();
                            claw.setPosition(closed);
                            extendState = ExtendState.TRANSFER;
                        }
                    }


                    break;
                case EJECT:
                    if (beamTimer.seconds() >= 1.5 && cBeam.getState()) {
                        collection.setPower(0);
                        extendState = ExtendState.EXTEND;
                    }
                    collection.setPower(-.4);
                    if (beamTimer.seconds() >= 1.64) {
                        collection.setPower(.75);
                    }
                    if (beamTimer.seconds() >= 2.2) {
                        extendState = ExtendState.EXTEND;
                    }

                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
                    break;

                case REJECT:
                        if (beamTimer.seconds() >= .4 && cBeam.getState()) {
                            collection.setPower(.6);
                            extendState = ExtendState.EXTEND;
                        }
                    break;
                case TRANSFER:
                    target = shortPos;
                    if (transferTimer.seconds() >= .1) {
                        extend.setPower(0);
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        claw.setPosition(closed);
                        deliveryS.setPosition(midPos);
                        return false;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);
            if (!dBeam.getState()) {
                theOpMode.telemetry.addData("Beam", "Broken");
            } else {
                theOpMode.telemetry.addData("Beam", "Not Broken");
            }
            theOpMode.telemetry.update();
            return true;
        }

    }

    public Action subCollect() {
        return new SubCollect();
    }

    public class MegaExtend implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                case START:
                    extendState = ExtendState.EXTEND;
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                case EXTEND:
                    target = extended;
                    if (Math.abs(extend.getCurrentPosition() - target) < 20) {
                        extend.setPower(0);
                        extendState = ExtendState.START;
                        return false;
                    }
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);

            extend.setPower(power);
            return true;
        }
    }

    public Action megaExtend() {
        return new MegaExtend();
    }




















































































    public class SubCollectBlue implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.15);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    deliveryS.setPosition(midPos);


                    break;
                // Rotate to collecting position
                case EXTEND:
                    target = extended;

                    if (Math.abs(extend.getCurrentPosition() - target) < 350) {
                        rCollection.setPosition(collect);
                        lCollection.setPosition(collect);
                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
                    break;
                case EXTENDED:
                    door.setPosition(.15);
                    target = extended;
                    if (beamTimer.seconds() > 0.8) {
                        target = mid;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            target = extended;
                            beamTimer.reset();
                        }
                        if (failTimer.seconds() >= 2.8) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            deliveryS.setPosition(midPos);
                            return false;
                        }
                    }

                    // If we collect a sample, retract extension, rotate to transfer position, stop collection
                    if (!cBeam.getState()) {
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.RETRACT;
                        beamTimer.reset();


                    }
                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
                    break;
                case RETRACT:
                    if (((double) colorSensor.red() / colorSensor.blue()) >= 1.4 && ((double) colorSensor.red() / colorSensor.alpha() >= 1.2) && (Math.abs(extend.getCurrentPosition())  >= 70)) {
                        collection.setPower(-.45);
                        extendState = ExtendState.REJECT;
                    }

                    claw.setPosition(open);
                    if (beamTimer.seconds() >= 1.2 && cBeam.getState()) {
                        extendState = ExtendState.EXTEND;
                    }
                    if (beamTimer.seconds() >= 1.5 && !cBeam.getState()) {
                        extendState = ExtendState.EJECT;
                    }

                    if (Math.abs(extend.getCurrentPosition() - retracted) < 50) {
                        deliveryS.setPosition(transferPos);
                        rCollection.setPosition(transfer);
                        lCollection.setPosition(transfer);
                        if (Math.abs(extend.getCurrentPosition() - retracted) < 22) {
                            door.setPosition(.4);
                            collection.setPower(.5);

                        }


                        if (!dBeam.getState()) {
                            door.setPosition(.15);
                            rCollection.setPosition(xHeight);
                            lCollection.setPosition(xHeight);
                            collection.setPower(-.95);
                            transferTimer.reset();
                            claw.setPosition(closed);
                            extendState = ExtendState.TRANSFER;
                        }
                    }


                    break;
                case EJECT:
                    if (beamTimer.seconds() >= 1.5 && cBeam.getState()) {
                        collection.setPower(0);
                        extendState = ExtendState.EXTEND;
                    }
                    collection.setPower(-.4);
                    if (beamTimer.seconds() >= 1.64) {
                        collection.setPower(.75);
                    }
                    if (beamTimer.seconds() >= 2.2) {
                        extendState = ExtendState.EXTEND;
                    }

                    if (!dBeam.getState()) {
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        collection.setPower(-.95);
                        transferTimer.reset();
                        claw.setPosition(closed);
                        extendState = ExtendState.TRANSFER;
                    }
                    break;
                case REJECT:
                    if (beamTimer.seconds() >= .4 && cBeam.getState()) {
                        collection.setPower(.6);
                        extendState = ExtendState.EXTEND;
                    }
                    break;
                case TRANSFER:
                    target = shortPos;
                    if (transferTimer.seconds() >= .1) {
                        extend.setPower(0);
                        collection.setPower(0);
                        target = retracted;
                        extendState = ExtendState.START;
                        claw.setPosition(closed);
                        deliveryS.setPosition(midPos);
                        return false;
                    }
                    break;

                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);
            if (!dBeam.getState()) {
                theOpMode.telemetry.addData("Beam", "Broken");
            } else {
                theOpMode.telemetry.addData("Beam", "Not Broken");
            }
            theOpMode.telemetry.update();
            return true;
        }

    }

    public Action subCollectBlue() {
        return new SubCollectBlue();
    }


































    public class SpecCollect implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.15);
                    collection.setPower(0);
                    extendState = ExtendState.EXTEND;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);

                    break;
                // Rotate to collecting position
                case EXTEND:
                    target = extended;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);

                    if (!cBeam.getState()) {
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.START;
                        return false;
                    }

                    if (Math.abs(extend.getCurrentPosition() - target) < 300) {

                        collection.setPower(.95);
                        beamTimer.reset();
                        failTimer.reset();
                        extendState = ExtendState.EXTENDED;
                    }
                    break;
                case EXTENDED:
                    if (beamTimer.seconds() > 0.85) {
                        target = shortPos;
                        lCollection.setPosition(transfer);
                        rCollection.setPosition(transfer);
                        if (beamTimer.seconds() > 1.1) {
                            lCollection.setPosition(collect);
                            rCollection.setPosition(collect);
                            target = extended;
                            beamTimer.reset();
                        }
                        if (failTimer.seconds() >= 1.6) {
                            extend.setPower(0);
                            collection.setPower(0);
                            target = retracted;
                            extendState = ExtendState.START;
                            return false;
                        }
                    }

                    // If we collect a specimen, retract extension, rotate to transfer position, stop collection
                    if (!cBeam.getState()) {
                        deliveryS.setPosition(lowerMid);
                        target = retracted;
                        rCollection.setPosition(xHeight);
                        lCollection.setPosition(xHeight);
                        //target = retracted;
                        collection.setPower(0);
                        extendState = ExtendState.START;
                        beamTimer.reset();
                        return false;

                    }
                    break;
                default:
                    extendState = ExtendState.START;
            }
            controller.setPID(p, i, d);
            int curPos = extend.getCurrentPosition();
            double power = controller.calculate(curPos, target);
            extend.setPower(power);
            theOpMode.telemetry.addData("Current State", extendState);
            theOpMode.telemetry.addData("Beam timer", beamTimer);
            if (!dBeam.getState()) {
                theOpMode.telemetry.addData("Beam", "Broken");
            } else {
                theOpMode.telemetry.addData("Beam", "Not Broken");
            }
            theOpMode.telemetry.update();
            return true;
        }

    }

    public Action specCollect() {
        return new SpecCollect();
    }













    public class SpitOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                //Fully Retracted in transfer position
                case START:
                    door.setPosition(.15);
                    collection.setPower(-1);
                    beamTimer.reset();
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.RETRACT;
                    break;

                case RETRACT:
                    if (beamTimer.seconds() >= .11) {
                        collection.setPower(0);
                        extendState = ExtendState.START;
                        return false;
                    }
                    break;
                default:
                    extendState = ExtendState.START;
            }
            return true;
        }

    }

    public Action spitOut() {
        return new SpitOut();
    }



    public class SweeperIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sweep.setPosition(in);
            return false;
        }
    }
    public Action sweeperIn() {
        return new SweeperIn();
    }


    public class SweeperOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sweep.setPosition(out);
            return false;
        }
    }
    public Action sweeperOut() {
        return new SweeperOut();
    }



    public class DropSample implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            switch (extendState) {
                case START:
                    beamTimer.reset();
                    deliveryS.setPosition(.8);
                    extendState = ExtendState.EXTEND;
                    break;
                case EXTEND:
                    if (beamTimer.seconds() >= .2) {
                        claw.setPosition(open);
                    }
                    if (beamTimer.seconds() >= 1) {
                        extendState = ExtendState.START;
                        return false;
                    }
                    break;

                default: extendState = ExtendState.START;
            }
            return true;

        }
    }
    public Action dropSample() {
        return new DropSample();
    }


























    public void IURITele() {
        if (theOpMode.gamepad2.a) {
            hang.setPower(.8);
            hang2.setPower(-.8);
        }
        else if (theOpMode.gamepad2.b) {
            hang.setPower(-.8);
            hang2.setPower(.8);
        }
        else {
            hang.setPower(0);
            hang2.setPower(0);
        }
        if (theOpMode.gamepad1.dpad_right) {
            sweep.setPosition(in);
        }
        else if (theOpMode.gamepad2.right_bumper) {
            sweep.setPosition(in);
        }
        else if (theOpMode.gamepad2.left_bumper) {
            sweep.setPosition(out);
        }
        if (theOpMode.gamepad2.dpad_up) {
            door.setPosition(.4);
        }
        else if (theOpMode.gamepad2.dpad_down) {
            door.setPosition(.15);
        }

        if (theOpMode.gamepad1.left_bumper || theOpMode.gamepad2.a) {
            failTimer.reset();
            claw.setPosition(open);
            // deliveryS.setPosition(backSpec);
            extendState = ExtendState.DELIVER;

        } else if (theOpMode.gamepad1.right_bumper || theOpMode.gamepad2.b) {
            claw.setPosition(closed);
        }
        if (theOpMode.gamepad1.dpad_up) {
            deliveryS.setPosition(backSpec);

        } else if (theOpMode.gamepad1.dpad_down) {
            deliveryS.setPosition(transferPos);
        }

        if (theOpMode.gamepad1.b) {
            extendState = ExtendState.EJECT;
        }
        if (theOpMode.gamepad1.ps) {
            extendState = ExtendState.RESET;
        }
        switch (extendState) {
            case DELIVER:
                if (failTimer.seconds() >= .3) {
                    deliveryS.setPosition(midPos);
                    extendState = ExtendState.START;
                }
                break;



            //Fully Retracted in transfer position
            case START:
                //sweep.setPosition(0);
                collection.setPower(0);
                rCollection.setPosition(xHeight);
                lCollection.setPosition(xHeight);

                if (theOpMode.gamepad1.dpad_left) {
                    deliveryS.setPosition(specPos);
                    claw.setPosition(open);
                    extendState = ExtendState.SPECIMEN;
                }
                // Start extending, turn on collection
                if (theOpMode.gamepad1.x) {
                    target = extended;
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.EXTEND;
                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.95);
                }


                break;
            case RESET:
                target = -600;
                if (!touch.getState()) {
                    deliveryS.setPosition(midPos);
                    extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    target = 0;
                    extendState = ExtendState.START;
                }
                break;
            // Rotate to collecting position

            case SPECIMEN:
                deliveryS.setPosition(specPos);
                claw.setPosition(open);
                if (!dBeam.getState()) {
                    transferTimer.reset();
                    claw.setPosition(specClosed);
                    extendState = ExtendState.SPECIMEN2;
                }
                break;

            case SPECIMEN2:
                if (transferTimer.seconds() >= .4) {
                    deliveryS.setPosition(transferPos);
                    extendState = ExtendState.START;
                }
                break;

            case EXTEND:
                target = extended;
                if (Math.abs(extend.getCurrentPosition() - extended) < 180 && theOpMode.gamepad1.x) {
                    extendState = ExtendState.EXTENDED;
                    collection.setPower(.95);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                }
                if (!dBeam.getState()) {
                    door.setPosition(.15);
                    collection.setPower(-.9);
                    transferTimer.reset();
                    claw.setPosition(closed);
                    extendState = ExtendState.TRANSFER;
                    theOpMode.telemetry.addData("DBeam", "Broken");
                    theOpMode.telemetry.update();
                }

                break;
            case EXTENDED:
            case MID:

                // If we collect a specimen, retract extension, rotate to transfer position, stop collection
                collection.setPower(.95);
                door.setPosition(.15);
                rCollection.setPosition(collect);
                lCollection.setPosition(collect);

                if (!cBeam.getState()) {
                    deliveryS.setPosition(lowerMid);
                    target = retracted;
                    collection.setPower(.3);
                    rCollection.setPosition(xHeight);
                    lCollection.setPosition(xHeight);
                    extendState = ExtendState.RETRACT;
                    theOpMode.telemetry.addData("Beam", "Broken");

                }
                if (theOpMode.gamepad1.y) {
                    target = mid;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.MID;
                    collection.setPower(.95);
                }
                if (theOpMode.gamepad1.x) {
                    target = extended;
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.EXTENDED;
                    collection.setPower(.95);

                }
                break;
            case RETRACT:


                claw.setPosition(open);
                if (Math.abs(extend.getCurrentPosition() - retracted) < 100) {
                    deliveryS.setPosition(transferPos);
                    rCollection.setPosition(transfer);
                    lCollection.setPosition(transfer);
                    collection.setPower(0);
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 70) {
                        door.setPosition(.4);
                    }
                    if (Math.abs(extend.getCurrentPosition() - retracted) < 40 && !liftTouch.getState()) {
                        collection.setPower(.5);
                    }
                }
                if (!dBeam.getState()) {
                    door.setPosition(.15);
                    collection.setPower(-1);
                    transferTimer.reset();
                    claw.setPosition(closed);
                    extendState = ExtendState.TRANSFER;
                    theOpMode.telemetry.addData("DBeam", "Broken");
                    theOpMode.telemetry.update();
                }

                break;
            case TRANSFER:

                theOpMode.telemetry.addData("transferTimer", transferTimer);
                theOpMode.telemetry.update();
                target = shortPos;
                if (transferTimer.seconds() >= .15) {
                    collection.setPower(0);
                    target = retracted;
                    claw.setPosition(closed);
                    deliveryS.setPosition(backSpec);
                    extendState = ExtendState.START;
                }
                break;
            case EJECT:
                if (theOpMode.gamepad1.b) {
                    collection.setPower(-.45);
                } else {
                    collection.setPower(0);
                    extendState = ExtendState.START;
                }
                break;
            case REJECT:
                target = extended;
                collection.setPower(-.6);
                if (Math.abs(extend.getCurrentPosition() - extended) < 50 && cBeam.getState()) {
                    collection.setPower(.95);
                    rCollection.setPosition(collect);
                    lCollection.setPosition(collect);
                    extendState = ExtendState.EXTENDED;
                }
                break;

            default:
                extendState = ExtendState.START;
        }
        if (theOpMode.gamepad1.a && extendState != ExtendState.START) {
            rCollection.setPosition(transfer);
            lCollection.setPosition(transfer);
            collection.setPower(0);
            target = retracted;
            extendState = ExtendState.START;
        }
        controller.setPID(p, i, d);
        int curPos = extend.getCurrentPosition();
        double power = controller.calculate(curPos, target);
        extend.setPower(power);
        // theOpMode.telemetry.addData("pos", curPos);
        theOpMode.telemetry.addData("target", target);
        theOpMode.telemetry.addData("Current State", extendState);
        theOpMode.telemetry.addData("power", power);
        theOpMode.telemetry.addData("Current Pos", curPos);
//        theOpMode.telemetry.addData("delivery position", deliveryS.getPosition());
        if (!dBeam.getState()) {
            theOpMode.telemetry.addData("Delivery Beam", "Broken");
        } else {
            theOpMode.telemetry.addData("Delivery Beam", "Not Broken");
        }
        if (!cBeam.getState()) {
            theOpMode.telemetry.addData("Collection Beam", "Broken");
        } else {
            theOpMode.telemetry.addData("Collection Beam", "Not Broken");
        }
        theOpMode.telemetry.update();
    }

}
