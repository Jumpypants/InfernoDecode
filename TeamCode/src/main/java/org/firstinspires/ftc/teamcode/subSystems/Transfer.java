package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transfer {
    private final Servo rightServo;
    private final Servo leftServo;
    public static double LEFT_UP_POS = 1;
    public static double RIGHT_UP_POS = 1;
    public static double LEFT_DOWN_POS = 1;
    public static double RIGHT_DOWN_POS = 1;


    public Transfer(HardwareMap hardwareMap){
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
    }


    public class MoveLeftTask extends Task {
        private final double pos;
        private final double estimatedTimeTaken;
        public MoveLeftTask(RobotContext robotContext, double pos) {
            super(robotContext);
            this.pos = pos;
            double currentPosition = leftServo.getPosition();
            double TIME_COEFFICIENT = 0.5;
            estimatedTimeTaken = Math.abs(pos - currentPosition) * TIME_COEFFICIENT;
        }

        public void initialize(RobotContext robotContext){
            leftServo.setPosition(pos);
        }

        protected boolean run(RobotContext robotContext){
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }

    }

    public class MoveRightTask extends Task {
        private final double pos;
        private final double estimatedTimeTaken;
        public MoveRightTask(RobotContext robotContext, double pos) {
            super(robotContext);
            this.pos = pos;
            double currentPosition = rightServo.getPosition();
            double TIME_COEFFICIENT = 0.5;
            estimatedTimeTaken = Math.abs(pos - currentPosition) * TIME_COEFFICIENT;
        }

        public void initialize(RobotContext robotContext){
            rightServo.setPosition(pos);

        }

        protected boolean run(RobotContext robotContext){
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }

    }
}
