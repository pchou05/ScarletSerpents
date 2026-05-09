package org.firstinspires.ftc.teamcode;

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.CRServo;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.util.ElapsedTime;

private DcMotor intakeMotor1;

public class routines extends LinearOpMode {
    intakeMotor1 = hardwareMap.DcMotor.get(DcMotor.class, "intakeMotor1");
    
    public static void intake(){
        intakeMotor1.setPower(1);
    }

    public void RPMControlInitialized()
    {
         double pastPosition = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double currentRPM = 0;
    
    private DcMotor shooterMotor;
    // private CRServo Servo1;
    // private CRServo Servo2;
    // private DcMotor intakeMotor1;
    // private DcMotor intakeMotor2;
    private String shooterState = "idle";
    private double offset = 0;
    
    private PIDController shooterPID;
    private boolean useRPMControl = true;
    
    @Override
            
    }
     public void runOpMode() throws InterruptedException {
        shooterMotor = hardwareMap.dcMotor.get("shooterMotor");
        // Servo1 = hardwareMap.crservo.get("Servo1");
        // Servo2 = hardwareMap.crservo.get("Servo2"); 
        // intakeMotor1 = hardwareMap.dcMotor.get("intakeMotor1");
        // intakeMotor2 = hardwareMap.dcMotor.get("intakeMotor2");
        
        // Initialize PID controller with tuned gains
        shooterPID = new PIDController(0.079, 0.0001, 0.001);

        waitForStart();

        if (isStopRequested()) return;

     while (opModeIsActive()) {
            updateRPM();
            
            // Toggle RPM control mode with dpad
            if(gamepad1.dpad_up) {
                useRPMControl = true;
                shooterPID.reset();
            } else if(gamepad1.dpad_down) {
                useRPMControl = false;
            }

            
            handleShooter(shooterState);
              if(gamepad2.right_bumper) {
                offset += 0.001;
            } else if(gamepad2.left_bumper) {
                offset -= 0.001;
            }
            
            // Display telemetry
            telemetry.addLine("=== SHOOTER INFO ===");
            telemetry.addData("Control Mode", useRPMControl ? "RPM CONTROL" : "POWER CONTROL");
            telemetry.addData("Shooter State", shooterState);
            telemetry.addData("Shooter RPM", "%.0f", currentRPM);
            telemetry.addData("Target RPM", "%.0f", getTargetRPM(shooterState));
            telemetry.addData("RPM Error", "%.0f", getTargetRPM(shooterState) - currentRPM);
            telemetry.addData("Shooter Power", "%.2f", shooterMotor.getPower());
            telemetry.addData("Power Offset (Trim)", "%.3f", offset);
            telemetry.addData("Shooter Encoder", shooterMotor.getCurrentPosition());
            
            // telemetry.addLine("\n=== INTAKE & SERVOS ===");
            // telemetry.addData("Intake1 Power", "%.2f", intakeMotor1.getPower());
            // telemetry.addData("Intake2 Power", "%.2f", intakeMotor2.getPower());
            // telemetry.addData("Servo1 Power", "%.2f", Servo1.getPower());
            // telemetry.addData("Servo2 Power", "%.2f", Servo2.getPower());
            
            telemetry.addLine("\n=== CONTROLS ===");
            telemetry.addData("DPad Up", "RPM Control Mode");
            telemetry.addData("DPad Down", "Power Control Mode");
            telemetry.addData("DPad Left", "Idle (300 RPM)");
            telemetry.addData("A", "Corner Shot");
            telemetry.addData("B", "Far Shot");
            telemetry.addData("X", "Line Shot");
            telemetry.addData("Y", "Off");
            telemetry.addData("GP2 Bumpers", "Adjust Offset");
            
            telemetry.update();
        }
        private void updateRPM(){
        double elapsedTime = timer.seconds();
        double currentPosition = shooterMotor.getCurrentPosition();
        if (elapsedTime > 0.1){
            currentRPM = ((currentPosition - pastPosition) * 60 / 28) / elapsedTime;
            pastPosition = currentPosition;
            timer.reset();
        }
    }
    
        private double getRPM(){
        return currentRPM;
    }
    private double getTargetRPM(String state) {
        switch (state){
            case "line": return 3500;      // Medium distance shot
            case "far": return 4200;       // Far distance shot - highest RPM
            case "corner": return 3200;    // Corner shot - slightly lower
            case "idle": return 300;       // Idle/standby mode
            case "off": return 0;          // Motor off
            case "wall": return 0;         // Motor off
            default: return 0;
        }
    }
    private void hitRPM(double value){
        double rpm = getRPM();
        if (rpm > value + 500){
            shooterMotor.setPower(0.2);
        } else if(rpm < value - 500){
            shooterMotor.setPower(1);
        } else {
            shooterMotor.setPower(0.51);
        }
    }
    private class PIDController {
        private double kp;
        private double ki;
        private double kd;
        private double integral;
        private double previousError;
        private ElapsedTime pidTimer;
        
        public PIDController(double kp, double ki, double kd) {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.integral = 0;
            this.previousError = 0;
            this.pidTimer = new ElapsedTime();
        }
        
        public double calculate(double error) {
            double dt = pidTimer.seconds();
            pidTimer.reset();
            
            // Prevent integral windup
            integral += error * dt;
            if (integral > 100) integral = 100;
            if (integral < -100) integral = -100;
            
            double derivative = (dt > 0) ? (error - previousError) / dt : 0;
            previousError = error;
            
            return (kp * error) + (ki * integral) + (kd * derivative);
        }
        
        public void reset() {
            integral = 0;
            previousError = 0;
            pidTimer.reset();
        }
    

    }
}
  public void Intake()
    {

    }
    
    public void Shooter() 
    {

    }
}