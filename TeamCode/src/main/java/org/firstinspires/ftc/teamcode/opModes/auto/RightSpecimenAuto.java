package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;

@Autonomous(name = "RightSpecimenAuto", group = "Scoring Auto")
public class RightSpecimenAuto extends LinearOpMode {
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
                .strafeToConstantHeading( new Vector2d(-35.75, 48) )
                .strafeToConstantHeading(new Vector2d(-35.75,30))

                .splineToConstantHeading(new Vector2d(-47.6,17), Math.toRadians(180))
                .strafeToConstantHeading(new Vector2d( -47.6, 61))
                .strafeToLinearHeading(new Vector2d(-47.6, 48), Math.toRadians(180));

        TrajectoryActionBuilder getSecSpec = acquireThreeSamples.endTrajectory().fresh()
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-50, 71), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-40, 72), Math.toRadians(184));

        TrajectoryActionBuilder preScoreSec = getSecSpec.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-40, 60), Math.toRadians(180));

        TrajectoryActionBuilder scoreSecSpec = preScoreSec.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(new Vector2d(-5,30), Math.toRadians(180)), Math.toRadians(270));

        TrajectoryActionBuilder moveBack = scoreSecSpec.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(new Vector2d(-8,50),Math.toRadians(180)),Math.toRadians(90));

        TrajectoryActionBuilder getThirdSpec = moveBack.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-48, 71), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(-38, 72), Math.toRadians(184));

        TrajectoryActionBuilder preScoreThird = getThirdSpec.endTrajectory().fresh()
                .strafeToLinearHeading(new Vector2d(-40, 60), Math.toRadians(180));

        TrajectoryActionBuilder scoreThirdSpec = preScoreThird.endTrajectory().fresh()
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(new Vector2d(-8,30), Math.toRadians(180)), Math.toRadians(270));

        TrajectoryActionBuilder park = scoreThirdSpec.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-38, 70), Math.toRadians(180));

        Actions.runBlocking(
                new SequentialAction(
                        //First spec
                        specimenArmSubsystem.CloseClaw(),
                        specimenArmSubsystem.ScoreSpecimen()
                        ,
                        scoreFirstSpec.build(),
                        specimenArmSubsystem.OpenClaw(),
                        acquireThreeSamples.build(),
                        new SleepAction(0.2),
                        new ParallelAction(
                                specimenArmSubsystem.OpenClaw(),
                                specimenArmSubsystem.PutDown()
                        ),
                        //Sec spec
                        new ParallelAction(
                                getSecSpec.build(),
                                specimenArmSubsystem.OpenClaw(),
                                specimenArmSubsystem.WallPos()
                        ),
                        specimenArmSubsystem.CloseClaw(),
                        new SleepAction(0.3),
                        specimenArmSubsystem.LiftPos(),
                        preScoreSec.build(),
                        specimenArmSubsystem.ScoreSpecimen(),
                        scoreSecSpec.build(),
                        //Third specimen
                        new ParallelAction(
                                moveBack.build(),
                                specimenArmSubsystem.OpenClaw()
                        ),
                        new ParallelAction(
                                getThirdSpec.build(),
                                specimenArmSubsystem.WallPos()
                        ),
                        specimenArmSubsystem.CloseClaw(),
                        new SleepAction(0.3),
                        specimenArmSubsystem.LiftPos(),
                        preScoreThird.build(),
                        specimenArmSubsystem.ScoreSpecimen(),
                        scoreThirdSpec.build(),
                        //Park
                        specimenArmSubsystem.OpenClaw(),
                        park.build()
                )
        );
    }
}