package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;

@Autonomous(name = "RightSpecimenAuto", group = "Real")
public final class RightSpecimenAuto extends LinearOpMode {
    private SpecimenArmSubsystem specimenArmSubsystem;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(-7.125,63.25 , Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        this.specimenArmSubsystem = new SpecimenArmSubsystem(this);

        waitForStart();

        TrajectoryActionBuilder scoreFirstSpec = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(5,29), Math.toRadians(180));

        TrajectoryActionBuilder acquireThreeSamples = scoreFirstSpec.endTrajectory().fresh()
                .setTangent(Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-35.75, 45), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-35.75,16), Math.toRadians(225))

                .splineToConstantHeading(new Vector2d(-47.6,16),Math.toRadians(90))
                .splineToConstantHeading(new Vector2d( -47.6, 61),Math.toRadians(270))

                .splineToLinearHeading(new Pose2d(-56.75,16, Math.toRadians(180)),Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-56.75,61),Math.toRadians(180))

                .setTangent(Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-64, 18, Math.toRadians(180)), Math.toRadians(135))
                .splineToConstantHeading(new Vector2d(-62, 61),Math.toRadians(180));

        TrajectoryActionBuilder getSecSpec = acquireThreeSamples.endTrajectory().fresh()
                .setTangent(270)
                .splineToConstantHeading(new Vector2d(-50, 68),Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-40, 68));

        TrajectoryActionBuilder scoreSecSpec = getSecSpec.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-2,30), Math.toRadians(270));

        TrajectoryActionBuilder getThirdSpec = scoreSecSpec.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(new Vector2d(-8,50),Math.toRadians(180)),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-57, 71), Math.toRadians(90))
                .strafeToConstantHeading(new Vector2d(-45, 71));

        TrajectoryActionBuilder scoreThirdSpec = getThirdSpec.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-11,30), Math.toRadians(270));

        Actions.runBlocking(
                new SequentialAction(
                        //First spec
                        new ParallelAction(
                                specimenArmSubsystem.CloseClaw(),
                                specimenArmSubsystem.ScoreSpecimen()
                        ),
                        new SleepAction(0.1),
                        scoreFirstSpec.build(),
                        new ParallelAction(
                                specimenArmSubsystem.OpenClaw(),
                                specimenArmSubsystem.PutDown(),
                                acquireThreeSamples.build()
                        ),
                        //Sec spec
                        new ParallelAction(
                            getSecSpec.build(),
                            specimenArmSubsystem.OpenClaw(),
                            specimenArmSubsystem.WallPos()
                        ),
                        specimenArmSubsystem.CloseClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(
                                scoreSecSpec.build(),
                                specimenArmSubsystem.ScoreSpecimen()
                        ),
                        //Third specimen
                        specimenArmSubsystem.OpenClaw(),
                        new ParallelAction(
                                getThirdSpec.build(),
                                specimenArmSubsystem.WallPos(),
                                specimenArmSubsystem.OpenClaw()
                        ),
                        specimenArmSubsystem.CloseClaw(),
                        new SleepAction(0.1),
                        new ParallelAction(
                                scoreThirdSpec.build(),
                                specimenArmSubsystem.ScoreSpecimen()
                        )

                )
        );
    }
}