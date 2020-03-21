package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Navx{
    public  AHRS navX; 
    public  double heading;
    public  double angle;

    
     // static variable single_instance of type Singleton 
     private static Navx single_instance = null; 
  
     // variable of type String 
     public String s; 
   
     // private constructor restricted to this class itself 
     private Navx() 
     { 
        navX = new AHRS(SPI.Port.kMXP);
        heading = navX.getFusedHeading();
        angle = navX.getAngle();
     } 
   
     // static method to create instance of Singleton class 
     public static Navx getInstance() 
     { 
         if (single_instance == null){ 
             single_instance = new Navx(); 
         }
         return single_instance; 
     }

     public double getFuzedHeading() {
        this.heading = navX.getFusedHeading();
        return heading;
     }

     public double getAngle() {
         this.angle = navX.getAngle();
         return angle;
     }

     public void zeroHeading() {
         navX.zeroYaw();
     }
}