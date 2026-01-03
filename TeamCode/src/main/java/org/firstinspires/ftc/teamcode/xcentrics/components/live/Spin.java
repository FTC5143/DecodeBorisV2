//package org.firstinspires.ftc.teamcode.xcentrics.components.live;
//
//import static org.firstinspires.ftc.teamcode.xcentrics.components.live.SpinConfig.*;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.control.PIDFController;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.xcentrics.components.Component;
//import org.firstinspires.ftc.teamcode.xcentrics.robots.LiveRobot;
//import org.firstinspires.ftc.teamcode.xcentrics.robots.Robot;
//import org.firstinspires.ftc.teamcode.xcentrics.util.qus.CRServoQUS;
//@Configurable
//class SpinConfig{
//    private static final double CPR_MOTOR = 751.8;        // counts per motor shaft revolution
//    private static final double GEAR_RATIO = 1.0;         // plate revs per motor rev denominator (1.0 => 1:1)
//    private static final double CPR_PLATE = CPR_MOTOR / GEAR_RATIO;
//
//    private static final double TICKS_PER_SLOT = CPR_PLATE / 3.0; // 120°
//    public static final double TICKS_PER_60   = CPR_PLATE / 6.0; // 60°
//    public static int PRESENCE_APLHA_THRESHOLD = 5;
//
//    public static PIDFCoefficients spinPIDcoef = new PIDFCoefficients(0,0,0,0);
//
//}
//public class Spin extends Component {
//    private DcMotorEx encoder;
//    private CRServoQUS spin;
//    private ColorSensor color;
//    private PIDFController spinPID = new PIDFController(spinPIDcoef);
//    private int zeroCount = 0;      // encoder tick value for slot 0 at intake
//    private double accum = 0.0;     // cumulative (floating) encoder target; drift-free rounding
//    private int targetCounts = 0;   // last set RUN_TO_POSITION target
//    private int intakeIndex = 0;    // which absolute slot is at intake stop
//    private static LiveRobot robot;
//    private static int purple = 0; //number of purple
//    private static int green = 0; //number of green
//    public Spin(Robot robot,LiveRobot liveRobot) {
//        super(robot);
//        this.robot = liveRobot;
//    }
//
//    public void registerHardware(HardwareMap hwmap){
//        super.registerHardware(hwmap);
//        encoder = hwmap.get(DcMotorEx.class,"BL");
//        spin = new CRServoQUS(hwmap.get(CRServo.class,"Spin"));
//        color = hwmap.get(ColorSensor.class,"color");
//    }
//    public enum Ball {EMPTY,PURPLE,GREEN}
//
//    private final Ball[] slots = {Ball.EMPTY,Ball.EMPTY,Ball.EMPTY};
//    public void resetEncoder(){
//        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        zeroCount = 0;
//    }
//    public void setBalls(Ball b1,Ball b2,Ball b3){
//        slots[0] = b1;
//        slots[1] = b2;
//        slots[2] = b3;
//    }
//    public void startup(){
//        super.startup();
//        accum = 0;
//        spinPID.updatePosition(encoder.getCurrentPosition());
//        spin.queue_power(spinPID.run());
//    }
//
//    public void update(OpMode opMode){
//        super.update(opMode);
//        intakeOne();
//        spin.queue_power(spinPID.run());
//        spin.update();
//    }
//    public void updateTelemetry(Telemetry telemetry){
//        super.updateTelemetry(telemetry);
//        telemetry.addData("slots", "%s | %s | %s", slots[0], slots[1], slots[2]);
//        addData("SpinTarget:",targetCounts);
//        addData("intakeIndex",intakeIndex);
//        addData("SpinPose:", encoder.getCurrentPosition());
//
//    }
//
//    private int mod3(int i){
//        return ((i%3) + 3)%3;
//    }
//    private int stepForward60(){
//        accum += TICKS_PER_60;
//        return(int)Math.rint(zeroCount+accum);
//    }
//
//    private void move120NoLaunch(){
//        int mid = stepForward60();
//        goTo(mid);
//        int stop = stepForward60();
//        goTo(stop);
//    }
//    private void goTo(int target){
//        targetCounts = target;
//        spinPID.setTargetPosition(targetCounts);
//    }
//    public void ejectPattern(){
//        if(robot.getPattern() == Camera.Pattern.PPG){
//            robot.spin.ejectPPG();
//        } else if(robot.getPattern() == Camera.Pattern.PGP){
//            robot.spin.ejectPGP();
//        } else {
//            robot.spin.ejectGPP();
//        }
//    }
//    public Runnable shootPattern(){
//        return new Runnable() {
//            @Override
//            public void run() {
//                ejectPattern();
//            }
//        };
//    }
//
//    private Ball readColorAtIntake(){
//        if(!ballPresentAtIntake()) return Ball.EMPTY;
//
//        int r = color.red();
//        int g = color.green();
//        int b = color.blue();
//
//        boolean isPurple = (b > g + 5) && (r > g + 5);
//        boolean isGreen  = (g > r + 5) && (g > b + 5);
//
//        if (isGreen && !isPurple){
//            green++;
//            return Ball.GREEN;
//        }
//        if (isPurple && !isGreen) {
//            purple++;
//            return Ball.PURPLE;
//        }
//
//        // Fallback: choose dominant channel (simple, tune as needed)
//        if (g >= r && g >= b) return Ball.GREEN;
//        return Ball.PURPLE;
//    }
//    public boolean ballPresentAtIntake() {
//        int alpha = color.alpha();
//        return alpha > PRESENCE_APLHA_THRESHOLD;
//    }
//
//    private int getIndex(Ball ball){
//        for(int i = 0; i < 3; i++){
//            if(slots[i] == ball){
//                return i;
//            }
//        }
//        return -1;
//    }
//    private void ejectBall(Ball ...balls){
//        for(Ball b : balls){
//            ejectSlot(getIndex(b));
//        }
//    }
//    public void ejectPPG() {
//        if(purple == 2 && green == 1){
//            ejectBall(Ball.PURPLE,Ball.PURPLE,Ball.GREEN);
//        }
//    }
//    public void ejectPGP(){
//        if(purple==2 && green == 1){
//            ejectBall(Ball.PURPLE,Ball.GREEN,Ball.PURPLE);
//        }
//    }
//    public void ejectGPP(){
//        if(purple == 2 && green == 1){
//            ejectBall(Ball.GREEN,Ball.PURPLE,Ball.PURPLE);
//        }
//    }
//    /** Read color at intake, store it in the current intake slot, then rotate +120° without ejecting. */
//    public void intakeOne() {
//        if(ballPresentAtIntake()) {
//            Ball color = readColorAtIntake();
//            slots[intakeIndex] = color;
//            halt(0.3);
//            // Move +120° as two half-steps; do not clear any slot
//            move120NoLaunch();
//
//            // Next slot is now at intake
//            intakeIndex = mod3(intakeIndex + 1);
//        }
//    }
//
//    /** shoot a specific absolute slot (0..2): align, +60° to eject (clear), +60° to finish; update intakeIndex. */
//    private void ejectSlot(int slotIndex) {
//        // Align so that after a +60° half-step, 'slotIndex' will be at eject.
//        // Eject slot at +60° is (intakeIndex + 1) % 3 from the current intake stop.
//        // Therefore, we need intakeIndex == slotIndex - 1 (mod 3).
//        int deltaSlots = mod3(slotIndex - 1 - intakeIndex);
//        for (int k = 0; k < deltaSlots; k++) {
//            move120NoLaunch(); // align in 120° chunks, do NOT clear while aligning
//            intakeIndex = mod3(intakeIndex + 1);
//        }
//
//        // Now +60° puts 'slotIndex' at eject
//        int mid = stepForward60();
//        goTo(mid);
//
//        robot.turret.launch();
//        halt(0.5);
//        if(slots[slotIndex] == Ball.PURPLE){
//            purple--;
//        }else if(slots[slotIndex] == Ball.GREEN){
//            green--;
//        }
//
//        // Logical clear
//        slots[slotIndex] = Ball.EMPTY;
//
//        // Finish the other +60° to land on an intake stop
//        int nextStop = stepForward60();
//        goTo(nextStop);
//
//        // We advanced one full slot
//        intakeIndex = mod3(intakeIndex + 1);
//    }
//    public void shoot(){
//        for(int i = intakeIndex; i < intakeIndex + 2; i++){
//            if(slots[i%3] != Ball.EMPTY){
//                ejectSlot(i%3);
//            }
//        }
//    }
//
//}
