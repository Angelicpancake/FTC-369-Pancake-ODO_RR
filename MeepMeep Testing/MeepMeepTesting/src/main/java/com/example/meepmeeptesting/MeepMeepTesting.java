package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(1000);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 70, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -60, 3*Math.PI/2))
                .strafeTo(new Vector2d(-2,-26.5))
                .lineToY(-40)

                .strafeToConstantHeading(new Vector2d(54,-40))// move right
                .strafeToConstantHeading(new Vector2d(54,5))// move forward
                .strafeToConstantHeading(new Vector2d(70,0))// move right and a bit back
                .strafeToConstantHeading(new Vector2d(70,-49), new TranslationalVelConstraint(400))// push the first specimen down
                .strafeToConstantHeading(new Vector2d(70,0), new TranslationalVelConstraint(400))// go back up
                .strafeToConstantHeading(new Vector2d(80,0))// allign to the second sample
                .strafeToConstantHeading(new Vector2d(80,-49), new TranslationalVelConstraint(400)) // push the second down
                .strafeToConstantHeading(new Vector2d(80,0),new TranslationalVelConstraint(400))
                .strafeToConstantHeading(new Vector2d(94,0)) // go back up and to the right
                .strafeToConstantHeading(new Vector2d(94,-48), new TranslationalVelConstraint(400))  // ush the third down

                .strafeToConstantHeading(new Vector2d(94, -25))
                .strafeToLinearHeading(new Vector2d(40, -62), Math.PI/2)

                .strafeToLinearHeading(new Vector2d(-4,-23), Math.toRadians(90))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}