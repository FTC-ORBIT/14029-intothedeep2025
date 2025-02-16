//package org.firstinspires.ftc.teamcode.Autonomous;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.roadRunner_1_0.MecanumDrive;
//import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.Arm;
//import org.firstinspires.ftc.teamcode.robotSubSystems.Arm.ArmState;
//import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVertical;
//import org.firstinspires.ftc.teamcode.robotSubSystems.ElevatorVertical.ElevatorVerticalState;
//
//@Autonomous(name = "crisi the crispi")
//public class BasketAuto extends LinearOpMode {
//    final double robotCenterToArm = 10;
//    Pose2d yellow2GamePiece1 = RotatedPose2d.rotate90deg(new Pose2d(-68, -26 - robotCenterToArm,Math.toRadians(90)));
//    Pose2d yellow2GamePiece2 = RotatedPose2d.rotate90deg(new Pose2d(-58, -26 - robotCenterToArm,Math.toRadians(90)));
//    Pose2d yellow2GamePiece3 = RotatedPose2d.rotate90deg(new Pose2d(-48, -26 - robotCenterToArm,Math.toRadians(90)));
//    Pose2d redRobot2StartPosition = RotatedPose2d.rotate90deg(new Pose2d(-37,0,Math.toRadians(90)));
//    Pose2d redBasket = RotatedPose2d.rotate90deg(new Pose2d(-64 + robotCenterToArm/Math.sqrt(2), -64 + robotCenterToArm/Math.sqrt(2),Math.toRadians(225)));
//    Pose2d redGamePiece1 = RotatedPose2d.rotate90deg(new Pose2d(68 - robotCenterToArm, -26, Math.toRadians(0)));
//    Pose2d redGamePiece2 = RotatedPose2d.rotate90deg(new Pose2d(58 - robotCenterToArm, -26, Math.toRadians(0)));
//    Pose2d redGamePiece3 = RotatedPose2d.rotate90deg(new Pose2d(48 - robotCenterToArm, -26, Math.toRadians(0)));
//    Pose2d start = RotatedPose2d.rotate90deg(new Pose2d(0,0,Math.toRadians(90)));
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        ElevatorVertical.init(hardwareMap);
//        Arm.init(hardwareMap);
//
//        ElevatorVerticalState elevatorVerticalState = ElevatorVerticalState.INTAKE;
//        ArmState armState = ArmState.INTAKE;
//
//
//        MecanumDrive drive = new MecanumDrive(hardwareMap,(start));
//        Pose2d target = RotatedPose2d.rotate90deg(new Pose2d(20,0,Math.toRadians(90)));
////        Pose2d target = new Pose2d(-60,-64,Math.toRadians(270));
//
//
//        TrajectoryActionBuilder startToBasket = drive.actionBuilder(start)
//                .strafeToLinearHeading(redBasket.position,redBasket.heading)
//
//        ;
//        TrajectoryActionBuilder basketToFirstPiece = drive.actionBuilder(redBasket)
//                .strafeToLinearHeading(yellow2GamePiece3.position,yellow2GamePiece3.heading)
//                ;
//        TrajectoryActionBuilder firstPieceToBasket = drive.actionBuilder(yellow2GamePiece3)
//                .strafeToLinearHeading(redBasket.position,redBasket.heading)
//                ;
//        TrajectoryActionBuilder basketToSecondPiece = drive.actionBuilder(redBasket)
//                .strafeToLinearHeading(yellow2GamePiece2.position,yellow2GamePiece2.heading)
//                ;
//        TrajectoryActionBuilder secondPieceToBasket = drive.actionBuilder(yellow2GamePiece2)
//                .strafeToLinearHeading(redBasket.position,redBasket.heading)
//                ;
//        TrajectoryActionBuilder basketToThirdPiece = drive.actionBuilder(redBasket)
//                .strafeToLinearHeading(yellow2GamePiece1.position,yellow2GamePiece1.heading)
//                ;
//        TrajectoryActionBuilder thirdPieceToBasket = drive.actionBuilder(yellow2GamePiece1)
//                .strafeToLinearHeading(redBasket.position,redBasket.heading)
//                ;
//
//        public Action sample(){
//            //אלמוג
//
//        }
//
//        public Action highBasket(){
//            //
//            public boolean run(....){
//                if !initialized...
//                elevator.resetencoder
//            }
//            Elevator.operate(DepletePosition, gamePadVal=0, ...)
//            return Elevator.getCurrentPosition!=Elevator.getWantedPosition()
//            }
//            elevatorVerticalState = ElevatorVerticalState.DEPLETE;
//            armState = ArmState.DEPLETE;
//        }
//
//        public Action closeElevator(){
//
//
//
//
//
//
//                ;
//        waitForStart();
//        ElevatorVertical.operate(elevatorVerticalState, 0,0);
//        Arm.operate(armState);
//
//
//
//        Actions.runBlocking(new SequentialAction(
//                startToBasket,
//                highBasket(),
//                basketToFirstPiece,
//                sample(),
//                firstPieceToBasket,
//                highBasket(),
//                basketToSecondPiece,
//                sample(),
//                secondPieceToBasket,
//                highBasket(),
//                basketToThirdPiece,
//                sample(),
//                thirdPieceToBasket,
//                highBasket()
//                )
//
//                .build());\
//
// */
//    }
//
//
//}
