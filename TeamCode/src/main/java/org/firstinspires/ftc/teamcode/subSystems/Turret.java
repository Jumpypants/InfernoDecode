package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {

    public static final int TURRET_MAX_POS = 5;
    public static final int TURRET_MIN_POS = 0;

    private final DcMotorEx turretMotor;

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private int limit(int value, int min, int max) {
        return Math.min(Math.max(value, min), max);
    }

    public void setTurretTargetPosition(int targetPosition, double power) {
        int clippedPos = limit(targetPosition, TURRET_MIN_POS, TURRET_MAX_POS);
        turretMotor.setTargetPosition(clippedPos);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(Math.abs(power));
    }

    public int getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    public void stopTurret() {
        turretMotor.setPower(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class RotateTurretTask extends Task {
        private final int TARGET_POSITION;
        private final double POWER;
        private final double estimatedTimeTaken;

        public RotateTurretTask(RobotContext robotContext, int targetPosition, double power) {
            super(robotContext);
            this.TARGET_POSITION = limit(targetPosition, TURRET_MIN_POS, TURRET_MAX_POS);
            this.POWER = Math.abs(power);
            this.estimatedTimeTaken = 5.0;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            turretMotor.setTargetPosition(TARGET_POSITION);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(POWER);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            boolean stillRunning = turretMotor.isBusy() && ELAPSED_TIME.seconds() < estimatedTimeTaken;
            if (!stillRunning) {
                stopTurret();
            }
            return stillRunning;
        }
    }
}
