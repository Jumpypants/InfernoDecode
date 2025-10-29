package org.firstinspires.ftc.teamcode.subSystems;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    public static final double TURRET_MAX_POS1 = 1;
    public static final double TURRET_MIN_POS1 = 0;

    public static final double TURRET_MAX_POS = 1;
    public static final double TURRET_MIN_POS = 0;


    private final Servo turretServo;
    private final Servo turretServo2;

    //servo1
    public Turret (HardwareMap hardwareMap) {
        this.turretServo = hardwareMap.get(Servo.class, "turretServo");
        this.turretServo2 = hardwareMap.get(Servo.class, "turretServo2");

    }

    private double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    private void setTurretPos(double pos) {
        this.turretServo.setPosition(limit(pos,TURRET_MIN_POS,TURRET_MAX_POS));
    }

    private void setTurretPos1(double pos) {
        this.turretServo2.setPosition(limit(pos,TURRET_MIN_POS1,TURRET_MAX_POS1));
    }

    public class RotateTurretTask extends Task {
        private final double TARGET_POSITION;
        private final double estimatedTimeTaken;

        /**
         * Creates a new Task with the provided RobotContext.
         *
         * @param robotContext contains references like telemetry, gamepads, and subsystems
         */

        public RotateTurretTask(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            TARGET_POSITION = targetPosition;
            this.estimatedTimeTaken = 5.0;
        }



        @Override
        protected void initialize(RobotContext robotContext) {
            turretServo.setPosition(TARGET_POSITION);
            turretServo2.setPosition(TARGET_POSITION);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }
    }

}
