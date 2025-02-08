package org.firstinspires.ftc.teamcode.opModes;

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

        TrajectoryActionBuilder getSecSpec = scoreFirstSpec.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-28,40 + 7.125), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-28, 18))
                //got in position
                .setTangent(270)
                .splineToConstantHeading(new Vector2d(-38, 17 + 7.125), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-38, 24 + 7.125), Math.toRadians(90))
                //controlled sample
                .strafeToConstantHeading(new Vector2d(-57, 71))
                .strafeToConstantHeading(new Vector2d(-45, 71));

        TrajectoryActionBuilder prepareScore1 = getSecSpec.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-45, 61));

        TrajectoryActionBuilder scoreSecSpec = getSecSpec.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-4,30), Math.toRadians(270));

        TrajectoryActionBuilder getThirdSpec = scoreSecSpec.endTrajectory().fresh()
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-28,40 + 7.125), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(-28, 18))
                .setTangent(270)
                //got in pos
                .splineToConstantHeading(new Vector2d(-47, 17 + 7.125), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-47, 24 + 7.125), Math.toRadians(90))
                //controlled sample
                .strafeToConstantHeading(new Vector2d(-57, 71))
                .strafeToConstantHeading(new Vector2d(-45, 71));

        TrajectoryActionBuilder prepareScore2 = getThirdSpec.endTrajectory().fresh()
                .strafeToConstantHeading(new Vector2d(-45, 61));

        TrajectoryActionBuilder scoreThirdSpec = prepareScore2.endTrajectory().fresh()
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-8,30), Math.toRadians(270));

        Actions.runBlocking(
                new SequentialAction(
                        //First spec
                        specimenArmSubsystem.CloseClaw(specimenArmSubsystem),
                        specimenArmSubsystem.ScoreSpecimen(specimenArmSubsystem),
                        scoreFirstSpec.build(),
                        //Sec spec
                        specimenArmSubsystem.OpenClaw(specimenArmSubsystem),
                        specimenArmSubsystem.WallPos(specimenArmSubsystem),
                        getSecSpec.build(),
                        specimenArmSubsystem.CloseClaw(specimenArmSubsystem),
                        specimenArmSubsystem.LiftPos(specimenArmSubsystem),
                        prepareScore1.build(),
                        specimenArmSubsystem.ScoreSpecimen(specimenArmSubsystem),
                        scoreSecSpec.build(),
                        //Third spec
                        specimenArmSubsystem.OpenClaw(specimenArmSubsystem),
                        specimenArmSubsystem.WallPos(specimenArmSubsystem),
                        getThirdSpec.build(),
                        specimenArmSubsystem.WallPos(specimenArmSubsystem),
                        prepareScore2.build(),
                        specimenArmSubsystem.ScoreSpecimen(specimenArmSubsystem),
                        scoreThirdSpec.build()

                )
        );
    }
}