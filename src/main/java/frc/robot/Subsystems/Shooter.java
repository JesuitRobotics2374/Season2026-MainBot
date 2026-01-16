// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
double speed; 
//private final (motor name) shooterMotor;
//private final (something) velocityController;
//private final relativeEncoder encoder; 
  /** Creates a new Shooter. */
  public Shooter() {

  }
/** Starts the shooter when ready to shoot
 * @param speed
 */
  public Command startShoot(double speed) {
    return null; 
    
  }
/** Stops the shooter when needed
 * @param speed
 */
  public Command stopShoot(double speed) {
    return null;
  }
/** Purges the shooter if balls get stuck
 * @param speed
 * @return
 */
  public Command purgeShooter(double speed) {
    return null;
    //
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
