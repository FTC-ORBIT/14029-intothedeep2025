package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizontical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizonticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.IntakeState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.Wrist;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.WristState;


@Autonomous(name = "AutoSample")
public class AutoSample extends LinearOpMode {
    public boolean isUp = true;
    public IntakeState robotIntakeState = IntakeState.OFF;
    WristState wristState = WristState.TRANSFER;

    ElevatorVerticalState lastelevatorVerticalState = ElevatorVerticalState.OFF;

    Pose2d redBasket = new Pose2d(-3, 25, Math.toRadians(-45));
    Pose2d sample1 = new Pose2d(-18/*-17.5*/, 27.5/*27*/, Math.toRadians(-12.8));//-13
    Pose2d sample2 = new Pose2d(-13, 18/*18*/, Math.toRadians(5));
    Pose2d sample3 = new Pose2d(-42.5, 8, Math.toRadians(22));
    Pose2d startPos = new Pose2d(0, 0, Math.toRadians(0));

    public ElevatorVerticalState robotVerticalElevatorState = ElevatorVerticalState.OFF;
    public ElevatorHorizonticalState robotHorizonticalElevatorState = ElevatorHorizonticalState.OFF;
    final double robotCenterToArm = 10;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        ElevatorVertical.init(hardwareMap);
        Arm.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        ElevatorHorizontical.init(hardwareMap);

        VelConstraint velConstraint = new TranslationalVelConstraint(90);
        AccelConstraint accelConstraint = new ProfileAccelConstraint(-50, 70);
        TrajectoryActionBuilder startToBasket = drive.actionBuilder(startPos)
                .strafeTo(redBasket.position , velConstraint, accelConstraint)
                .turnTo(redBasket.heading);
        TrajectoryActionBuilder basketToSample1 = drive.actionBuilder(redBasket)
                .turnTo(sample1.heading)
                .strafeTo(sample1.position);
        TrajectoryActionBuilder basketToSample2 = drive.actionBuilder(new Pose2d(-13, 16, redBasket.heading.toDouble()))
                .turnTo(sample2.heading)
                .strafeTo(new Vector2d(-23, 18));
        TrajectoryActionBuilder sample1ToBasket = drive.actionBuilder(sample1)
                .turnTo(redBasket.heading)
                .strafeToLinearHeading(new Pose2d(-9, 12, Math.toRadians(-45)).position, redBasket.heading);
        //.strafeTo(redBasket.position);
        TrajectoryActionBuilder sample2ToBasket = drive.actionBuilder(new Pose2d(-23,18, Math.toRadians(0)))
                .turnTo(redBasket.heading)
                .strafeTo(new Vector2d(-23, 4));
        //.strafeTo(redBasket.position);
        TrajectoryActionBuilder endTurn = drive.actionBuilder(new Pose2d(0, 18, Math.toRadians(-53)))
                .strafeTo(new Pose2d(-2, -10, Math.toRadians(0)).position)
                .turnTo(Math.toRadians(0));

        TrajectoryActionBuilder sample3ToBasket = drive.actionBuilder(sample3)
                .turnTo(redBasket.heading)
                .strafeTo(new Vector2d(-5, 4), velConstraint,accelConstraint
                );
        TrajectoryActionBuilder backFromBasket = drive.actionBuilder(new Pose2d(-9, 12, Math.toRadians(-45)))
                .strafeTo(new Vector2d(-13, 14.5));
        TrajectoryActionBuilder backFromBasket2 = drive.actionBuilder(new Pose2d(-23, 4, Math.toRadians(-45)))
                .strafeTo(new Vector2d(-27, 7));
        TrajectoryActionBuilder backFromBasket3 = drive.actionBuilder(new Pose2d(-5, 4, Math.toRadians(-45)))
                .strafeTo(new Vector2d(-5, 30));

        TrajectoryActionBuilder basketToSample3 = drive.actionBuilder(new Pose2d(-27, 7 , Math.toRadians(-45)))
                .turnTo(sample3.heading.toDouble())
                .strafeTo(sample3.position);


        waitForStart();


//        Actions.runBlocking(actionBuilder.build());
        //Actions.runBlocking(new SequentialAction(sampleIntake() , sampleTransfer()));
//        Actions.runBlocking(depleteAction()));
//        Actions.runBlocking(new SequentialAction(sampleIntake(),sampleTransfer(),sampleDeplete()));
        Actions.runBlocking(
                new SequentialAction(
                    new SequentialAction(
                        new ParallelAction(
                                    startToBasket.build()
                                    , elevatorDeplete(),
                            new SequentialAction( new SleepAction(2.75),
                                new ParallelAction(
                                        basketToSample1.build()
                                        , new SequentialAction(new SleepAction(0.5), sampleIntake(), sampleTransfer())
                                        , new SequentialAction(new SleepAction(5)
                                            , new ParallelAction(
                                                elevatorDepleteWithBack(backFromBasket)
                                                , sample1ToBasket.build() //-------------------------------------------
                                                , new SequentialAction( new SleepAction(3.2),
                                                    new ParallelAction(
                                                            basketToSample2.build()
                                                            , new SequentialAction(new SleepAction(0.4), sampleIntake(),sampleTransfer(),  armByState(ArmState.INTAKE) )
                                                            , new SequentialAction(new SleepAction(5.2)
                                                                ,new ParallelAction(
                                                                    elevatorDepleteWithBack(backFromBasket2)
                                                                    ,sample2ToBasket.build()
                                                                ), new ParallelAction(
                                                                        basketToSample3.build()
                                                                        , new SequentialAction(new SleepAction(0.4), sampleIntake(), armByState(ArmState.INTAKE) ,sampleTransfer())
                                                                        , new SequentialAction(new SleepAction(5.6)
                                                                        ,new ParallelAction(
                                                                            elevatorDepleteWithBack(backFromBasket3)
                                                                            ,sample3ToBasket.build()
                                                                        )
                                                                    )
                                                                )
                                                            )
                                                    )
                                                )

                                            )
                                        )
                                )
                            )
                        )
                    )
                )
        );
        AngleStorage.angle = (float) Math.toDegrees(drive.pose.heading.toDouble());
    }



    public Action elevatorDeplete() {
        return new SequentialAction(
                elevatorVericalByState(ElevatorVerticalState.DEPLETE, true),
                new ParallelAction(
                        elevatorVericalByState(ElevatorVerticalState.DEPLETE, false),
                        new SequentialAction(
                                new SleepAction(0.4),
                                armByState(ArmState.DEPLETE),
                                armByState(ArmState.INTAKE),

                                elevatorVericalByState(ElevatorVerticalState.INTAKE, true)
                        ),
                        wristByState(WristState.INTAKE)
                )
        );
    }

    public Action elevatorDepleteWithBack(TrajectoryActionBuilder back) {
        return new SequentialAction(
                elevatorVericalByState(ElevatorVerticalState.DEPLETE, true),
                new ParallelAction(
                        elevatorVericalByState(ElevatorVerticalState.DEPLETE, false),
                        new SequentialAction(
                                new SleepAction(0.4),
                                armByState(ArmState.DEPLETE),
                                new ParallelAction(
                                        armByState(ArmState.INTAKE),
                                        back.build()
                                ),
                                elevatorVericalByState(ElevatorVerticalState.INTAKE, true)
                        ),
                        wristByState(WristState.INTAKE)
                )
        );
    }

    public Action sampleIntake() {
        return new SequentialAction(
                new ParallelAction(
                        elevatorHorizontalByState(ElevatorHorizonticalState.HALF, true),
                        armByState(ArmState.INTAKE),
                        wristAction(WristState.INTAKE),
                        intakeByState(IntakeState.IN)
                ), new SleepAction(0.8)
        );
    }

    public Action sampleTransfer() {

        return new ParallelAction(
                elevatorVericalByState(ElevatorVerticalState.INTAKE, false),
                armByState(ArmState.INTAKE),
                new SequentialAction(
                        new ParallelAction(
                                wristAction(WristState.TRANSFER)
                                , intakeByState(IntakeState.OFF)
                        )
                        , new ParallelAction(
                            elevatorHorizontalByState(ElevatorHorizonticalState.CLOSE, false)
                        , new SequentialAction(
                            new SleepAction(0.5) ,intakeByState(IntakeState.OUT)
                        )
                        , new SequentialAction(
                        new SleepAction(0.1), intakeByState(IntakeState.OFF)
                )
                        , elevatorVericalByState(ElevatorVerticalState.OFF, true)
                )

                ));
    }

    public Action elevatorVericalByState(final ElevatorVerticalState elevatorVerticalState, boolean stopAfterAction) {
        return new Action() {
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    lastelevatorVerticalState = elevatorVerticalState;
                    initialized = true;
                }
                ElevatorVertical.operate(elevatorVerticalState, 0, 0);
                telemetryPacket.put("elevatorVerticalState", elevatorVerticalState);
                telemetryPacket.put("lastElevatorVerticalState", lastelevatorVerticalState);
                telemetryPacket.put("elevator pos", ElevatorVertical.getElevatorPos());
                telemetryPacket.put("wantedPos", ElevatorVertical.getWantedPos());
                telemetryPacket.put("inPos", ElevatorVertical.inPos());
                if (ElevatorVertical.getElevatorPos() > 400 && ElevatorVertical.getElevatorPos() < 2250) {
                    Arm.operate(ArmState.HALF);
                }
                if (elevatorVerticalState == ElevatorVerticalState.OFF) {
                    return false;
                }
                if (stopAfterAction) {
                    return !ElevatorVertical.inPos();
                } else {
                    return lastelevatorVerticalState == elevatorVerticalState;
                }
            }
        };
    }

    public Action elevatorHorizontalByState(final ElevatorHorizonticalState elevatorHorizonticalState, boolean isOpening) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElevatorHorizontical.opreate(elevatorHorizonticalState, 0, false);
                telemetryPacket.put("elevatorHorizontalState", elevatorHorizonticalState);
                telemetryPacket.put("elevatorHpos", ElevatorHorizontical.getElevatorPos());
                telemetryPacket.put("wantedHPos", ElevatorHorizontical.getWantedPos());
                telemetryPacket.put("inPosH", ElevatorHorizontical.inPos());
                if (isOpening) {
                    return !ElevatorHorizontical.atLeastPose();
                }
                return !ElevatorHorizontical.inPos();
            }
        };
    }


    public Action armByState(ArmState armState) {
        Action armAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.operate(armState);
                telemetryPacket.put("arm position", Arm.armServo.getPosition());
                return false;
            }

        };
        return new SequentialAction(armAction, new SleepAction(1));
    }

    public Action wristByState(final WristState wristState) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Wrist.operate(wristState);
                return false;
            }
        };
    }

//    public Action intakeByState(IntakeState intakeState){
//        return new Action() {
//            private boolean isInitialized = false;
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                if (!isInitialized){
//                    robotIntakeState = intakeState;
//                    isInitialized = true;
//                }
//                Intake.operate(robotIntakeState);
//                return robotIntakeState != IntakeState.OFF;
//            }
//
//        };
//    }


    public Action intakeByState(IntakeState intakeState) {
        Action servoIntakeAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                telemetryPacket.put("leftWrist", Intake.leftIntakeServo.getPosition());
                Intake.operate(intakeState);
                if (intakeState == IntakeState.IN) {
                    return Intake.rightIntakeServo.getPosition() < Intake.POS_IN_RIGHT_INTAKE;
                } else if (intakeState == IntakeState.OFF) {
                    return Intake.rightIntakeServo.getPosition() < Intake.POS_OFF_RIGHT_INTAKE;
                } else {
                    return Intake.rightIntakeServo.getPosition() < Intake.POS_OUT_RIGHT_INTAKE;
                }
            }
        };
        return new SequentialAction(servoIntakeAction, new SleepAction(1));

    }

    public Action wristAction(WristState wristState) {
        Action servoWristAction = new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                telemetryPacket.put("rightServoPos",Wrist.rightWristServo.getPosition());
                Wrist.operate(wristState);
                if (wristState == WristState.INTAKE) {
                    return (Wrist.rightWristServo.getPosition() < Wrist.POS_INTAKE_RIGHT);
                } else if (wristState == WristState.DEPLETE) {
                    return Wrist.rightWristServo.getPosition() < Wrist.POS_DEPLETE_RIGHT;
                } else {
                    return Wrist.rightWristServo.getPosition() < Wrist.POS_TRANSFER_RIGHT;
                }
            }
        };
        return new SequentialAction(servoWristAction, new SleepAction(1));

    }

    public Action intake() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                elevatorHorizontalByState(ElevatorHorizonticalState.HALF, true);
                wristByState(WristState.INTAKE);
                intakeByState(IntakeState.IN);
                return Intake.leftIntakeServo.getPosition() <= Intake.POS_OUT_LEFT_INTAKE && Wrist.leftWristServo.getPosition() <= Wrist.POS_INTAKE_LEFT && !ElevatorHorizontical.inPos();
            }
        };
    }

    //
    public Action driveAction(Pose2d start, Pose2d end, MecanumDrive drivetrain) {
        TrajectoryActionBuilder drive = drivetrain.actionBuilder(start)
                .turnTo(end.heading)
                .strafeToLinearHeading(end.position, end.heading);
        return drive.build();

    }

    public void printCurrentPos(MecanumDrive drive) {
        drive.updatePoseEstimate();
        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();
        while (!gamepad1.a) ;
    }


};



