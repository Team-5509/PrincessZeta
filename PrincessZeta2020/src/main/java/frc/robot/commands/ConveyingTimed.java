package frc.robot.commands;
public class ConveyingTimed extends Index {
    public ConveyingTimed(double spinInVoltage) {
        super(spinInVoltage);
    }
    @Override
    public void initialize(){
        super.initialize();
        setTimeout(5);
    }
    @Override
    public boolean isFinished(){
        return super.isFinished() || isTimedOut();
    }
}