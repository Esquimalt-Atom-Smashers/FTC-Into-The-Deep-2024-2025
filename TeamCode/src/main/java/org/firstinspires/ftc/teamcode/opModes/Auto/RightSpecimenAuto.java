package org.firstinspires.ftc.teamcode.opModes.Auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
        Pose2d beginPose = new Pose2d(-7.125,63.25 , Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        this.specimenArmSubsystem = new SpecimenArmSubsystem(this);

        waitForStart();

        TrajectoryActionBuilder scoreFirstSpec = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(new Vector2d(-1,30), Math.toRadians(180));

        TrajectoryActionBuilder acquireThreeSamples = scoreFirstSpec.endTrajectory().fresh()

                .strafeToConstantHeading( new Vector2d(-35.75, 60) )
                .strafeToConstantHeading(new Vector2d(-35.75,30))

                .splineToConstantHeading(new Vector2d(-47.6,16),Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d( -47.6, 61))
                .setTangent(Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(-56.75,16),Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-56.75,61))

                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-62, 20), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d(-62, 61));

        TrajectoryActionBuilder getSecSpec = acquireThreeSamples.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-57, 72))
                .strafeToConstantHeading(new Vector2d(-45, 72));

        TrajectoryActionBuilder preScoreSecSpec = getSecSpec.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-45, 61));

        TrajectoryActionBuilder scoreSecSpec = preScoreSecSpec.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-4,30), Math.toRadians(270));

        TrajectoryActionBuilder getThirdSpec = scoreSecSpec.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-57, 71))
                .strafeToConstantHeading(new Vector2d(-45, 71));

        TrajectoryActionBuilder preScoreThirdSpec = getThirdSpec.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-45, 61));

        TrajectoryActionBuilder scoreThirdSpec = preScoreThirdSpec.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-8,30), Math.toRadians(270));

        Actions.runBlocking(
                new SequentialAction(
                        //First spec
                        new ParallelAction(
                                specimenArmSubsystem.CloseClaw(),
                                specimenArmSubsystem.ScoreSpecimen()
                        ),
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
                        new ParallelAction(
                                preScoreSecSpec.build(),
                                specimenArmSubsystem.LiftPos()
                        ),
                        new ParallelAction(
                                scoreSecSpec.build(),
                                specimenArmSubsystem.ScoreSpecimen()
                        ),
                        //Third specimen
                        specimenArmSubsystem.OpenClaw(),
                        new ParallelAction(
                                scoreThirdSpec.build(),
                                specimenArmSubsystem.WallPos(),
                                specimenArmSubsystem.OpenClaw()
                        ),
                        specimenArmSubsystem.CloseClaw(),
                        new ParallelAction(
                                preScoreThirdSpec.build(),
                                specimenArmSubsystem.LiftPos()
                        ),
                        new ParallelAction(
                                scoreThirdSpec.build(),
                                specimenArmSubsystem.ScoreSpecimen()
                        )
                )
        );
    }
}