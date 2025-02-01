package org.firstinspires.ftc.team26396.opmodes.auto.blue;


// RR-specific imports

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.team26396.opmodes.auto.presets.Arm;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.Claw;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.LinearSlide;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.Roll;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.XYaw;
import org.firstinspires.ftc.team26396.opmodes.auto.presets.YPitch;
import org.firstinspires.ftc.team26396.roadrunner.teamcode.MecanumDrive;

@Autonomous(name="Blue High Basket Drop Auto Path")
public class BlueHighBasketDropAutoPath extends LinearOpMode {
    public void runOpMode() {
        // I'm assuming you're at 0, 60, facing the basket
        Pose2d initialPose = new Pose2d(0, 60, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // Define arm positions using the constants from the Arm class

        Arm arm = new Arm(hardwareMap);
        LinearSlide linearSlide = new LinearSlide(hardwareMap);

        XYaw yaw = new XYaw(hardwareMap);
        YPitch pitch = new YPitch(hardwareMap);
        Roll roll = new Roll(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        waitForStart();

        double xDestPositionDropSampleInHand = 52;
        double yDestPositionDropSampleInHand = 50;
        double headingDestPositionDropSampleInHand = Math.toRadians(0.0);

        TrajectoryActionBuilder goToBasketFromInitPosition = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(xDestPositionDropSampleInHand, yDestPositionDropSampleInHand), Math.toRadians(45));


        Action sleepAction1Second = new SleepAction(1);
        Action sleepAction2Seconds = new SleepAction(2);
        Action sleepAction3Seconds = new SleepAction(3);

        TrajectoryActionBuilder basketToFirstSample =
                goToBasketFromInitPosition.endTrajectory().fresh()
                        .turnTo(Math.toRadians(-105))
                        .strafeTo(new Vector2d(48, 39));
//                        .lineToY(38);

        TrajectoryActionBuilder firstSampleToBasket =
                basketToFirstSample.endTrajectory().fresh()
                        .strafeToLinearHeading(new Vector2d(xDestPositionDropSampleInHand, yDestPositionDropSampleInHand), Math.toRadians(45));

        Action goToBasketFromInitPositionAction = goToBasketFromInitPosition.build();
        Action basketToFirstSampleAction = basketToFirstSample.build();
        Action firstSampleToBasketAction = firstSampleToBasket.build();

        Action fullAction = new SequentialAction(
                // Init position to basket with pre-loaded sample
                goToBasketFromInitPositionAction,
                // Drop the sample into the basket
                buildCommonActionForDroppingToBasket(goToBasketFromInitPosition, arm, linearSlide, claw, pitch),
                // Go from basket to the first sample
                basketToFirstSampleAction,
                // Pick up first sample
                buildCommonActionToPickSample(basketToFirstSample, arm, linearSlide, claw, pitch),
                // First sample to basket
                firstSampleToBasketAction,
                // Drop the sample into the basket
                buildCommonActionForDroppingToBasket(goToBasketFromInitPosition, arm, linearSlide, claw, pitch));
                // Go from basket to the second sample

        Actions.runBlocking(fullAction);

    }

    public Action buildCommonActionForDroppingToBasket(TrajectoryActionBuilder inputTrajectory, Arm arm, LinearSlide linearSlide, Claw claw, YPitch pitch) {

        Action dropSampleIntoHighBasketAction = inputTrajectory.endTrajectory().fresh()
//                .stopAndAdd(new SleepAction(1))
                // Raise the arm to upper basket height
                .stopAndAdd(arm.raiseArmForUpperBasket())
                .stopAndAdd(new SleepAction(1))
                // Extend the slide
                .stopAndAdd(linearSlide.extendArmForward())
                .stopAndAdd(new SleepAction(1))
                // Move towards the basket
                .lineToX(56)
                // Turn the wrist towards the basket - wrist up method may be misleading
                .stopAndAdd(pitch.moveWristUp())
                .stopAndAdd(new SleepAction(.5))
                // Open the claw to drop the sample
                .stopAndAdd(claw.openClaw())
                .stopAndAdd(new SleepAction(1))
                // Move the wrist back - wrist down method may be misleading
                .stopAndAdd(pitch.moveWristDown())
//                .stopAndAdd(new SleepAction(1))
                // Retract slide backward
                .lineToX(52)
                .stopAndAdd(new SleepAction(.5))
                .stopAndAdd(linearSlide.retractSlideBackward())
                .stopAndAdd(new SleepAction(2))
                // Bring the arm back down
                .stopAndAdd(arm.deactivateArm())
//                .stopAndAdd(new SleepAction(2))
                .build();

        return dropSampleIntoHighBasketAction;
    }

    public Action buildCommonActionToPickSample(TrajectoryActionBuilder inputTrajectory, Arm arm, LinearSlide linearSlide, Claw claw, YPitch pitch) {

        Action pickSampleFromFloor = inputTrajectory.endTrajectory().fresh()
                .stopAndAdd(pitch.moveWristUp())
//                .stopAndAdd(linearSlide.moveSlideRelatively(100))
                .stopAndAdd(claw.openClaw())
                .stopAndAdd(new SleepAction(1))
                .stopAndAdd(claw.closeClaw())
                .stopAndAdd(new SleepAction(1))
//                .stopAndAdd(linearSlide.moveSlideRelatively(-100))
                .build();

        return pickSampleFromFloor;
    }
}