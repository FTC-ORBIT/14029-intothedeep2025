package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmState;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalConstants;
import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;

@Autonomous(name = "test")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        ElevatorVertical.init(hardwareMap);
        Arm.init(hardwareMap);
        TrajectoryActionBuilder actionBuilder = drive.actionBuilder(new Pose2d(0,0,0)).
        strafeToLinearHeading(RotatedPose2d.rotate90deg(new Pose2d(-23,4,0)).position,0)
                .turnTo(Math.toRadians(-45));
        waitForStart();
        //Actions.runBlocking(actionBuilder.build());
        Actions.runBlocking(new ParallelAction(
                highBasket(),
                new SequentialAction(
                    armHalf(),new ParallelAction(new SleepAction(1),armDeplete())
                )
                )
        );
    }

    public Action highBasket(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                ElevatorVertical.operate(ElevatorVerticalState.DEPLETE, 0,0);
                return ElevatorVertical.getElevatorPos() <= ElevatorVerticalConstants.DepletePos -5;
            }

        };
    }
    public Action armDeplete(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                Arm.operate(ArmState.DEPLETE);
                telemetryPacket.put("arm position",Arm.armServo.getPosition());
                return Arm.armServo.getPosition() <= 0.74;
            }

        };
    }
    public Action armHalf(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (ElevatorVertical.getElevatorPos() > 200) {
                    Arm.operate(ArmState.HALF);
                }
                return Arm.armServo.getPosition() <= 0.36;
            }

        };
    }

}
