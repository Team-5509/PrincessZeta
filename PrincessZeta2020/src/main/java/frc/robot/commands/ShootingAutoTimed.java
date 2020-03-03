package frc.robot.commands;
public class ShootingAutoTimed extends ShootingAuto{
    public ShootingAutoTimed(){
        super();
    }
    @Override
    public void initialize(){
        super.initialize();
        setTimeout(9);
    }
    @Override
    public boolean isFinished(){
        return super.isFinished() || isTimedOut();
    }
}