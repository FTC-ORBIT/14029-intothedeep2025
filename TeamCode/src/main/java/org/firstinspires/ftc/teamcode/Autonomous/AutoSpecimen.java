package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorHorizontical.ElevatorHorizontical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;
import org.firstinspires.ftc.teamcode.robotSubSystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.robotSubSystems.Wrist.Wrist;

@Autonomous(name = "autoSpecimen")
public class AutoSpecimen extends LinearOpMode {
    ElevatorVerticalState lastelevatorVerticalState = ElevatorVerticalState.OFF;
    boolean isUp= true;boolean isUp1= true;
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,63,0));
        ElevatorVertical.init(hardwareMap);
        Arm.init(hardwareMap);
        Intake.init(hardwareMap);
        Wrist.init(hardwareMap);
        ElevatorHorizontical.init(hardwareMap);



        TrajectoryActionBuilder firstBuilder = drive.actionBuilder(new Pose2d(0, 63, 0)).
                strafeToLinearHeading(new Pose2d(32, 63, 0).position, 0);
        TrajectoryActionBuilder secondBuilder = drive.actionBuilder(new Pose2d(32, 63, 0)).
                strafeToLinearHeading(new Pose2d(10, 90, 0).position, 0)
                .turnTo(Math.toRadians(180));
        TrajectoryActionBuilder thirdBuilder = drive.actionBuilder(new Pose2d(10, 90, Math.toRadians(180))).
                strafeToLinearHeading(new Vector2d(0,105),Math.toRadians(180))
                .strafeToLinearHeading(new Vector2d(-5,105),Math.toRadians(180));
        TrajectoryActionBuilder fourthBuilder = drive.actionBuilder(new Pose2d(-5, 105, Math.toRadians(180))).
                strafeTo(new Pose2d(15, 63, Math.toRadians(180)).position)
                .turnTo(Math.toRadians(0))
                .strafeTo(new Pose2d(15,63,0).position)
                .strafeTo(new Pose2d(32,67,0).position);
        TrajectoryActionBuilder fifthBuilder = drive.actionBuilder(new Pose2d(0, -31, 0))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-35, -20, 0)).position, 0).turnTo(Math.toRadians(90)).turnTo(Math.toRadians(180))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-35, -50, 0)).position, Math.toRadians(180))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-50, -50, 0)).position, Math.toRadians(180))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-50, -6, 0)).position, Math.toRadians(180))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-50, -50, 0)).position, Math.toRadians(180))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-66, -50, 0)).position, Math.toRadians(180))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-66, -6, 0)).position, Math.toRadians(180))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-66, -50, 0)).position, Math.toRadians(180))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-75, -50, 0)).position, Math.toRadians(180))
                .strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-75, -6, 0)).position, Math.toRadians(180));

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        firstBuilder.build()
                        , elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN, true)
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                secondBuilder.build()
                                , elevatorVericalByState(ElevatorVerticalState.INTAKE,true)
                        )
                        ,new ParallelAction(
                                thirdBuilder.build()
                                , elevatorVericalByState(ElevatorVerticalState.SPECIMEN,true)
                        )
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        fourthBuilder.build(),
                        elevatorVericalByState(ElevatorVerticalState.PUTSPECIMEN,true)
                )
        );
    }
    public Action elevatorVericalByState(final ElevatorVerticalState elevatorVerticalState, boolean stopAfterAction){
        return new Action() {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    lastelevatorVerticalState = elevatorVerticalState;
                    initialized = true;
                }
                ElevatorVertical.operate(elevatorVerticalState,0,0);
                telemetryPacket.put("elevatorVerticalState",elevatorVerticalState);
                telemetryPacket.put("lastElevatorVerticalState",lastelevatorVerticalState);
                telemetryPacket.put("elevator pos", ElevatorVertical.getElevatorPos());
                telemetryPacket.put("wantedPos", ElevatorVertical.getWantedPos());
                telemetryPacket.put("inPos",ElevatorVertical.inPos());
                if (elevatorVerticalState ==ElevatorVerticalState.OFF) {
                    return false;
                }
                if (stopAfterAction) {
                    return !ElevatorVertical.inPos();
                }else{
                    return lastelevatorVerticalState==elevatorVerticalState;
                }
            }
        };
    }

}
