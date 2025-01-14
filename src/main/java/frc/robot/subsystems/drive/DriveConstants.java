package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveConstants {
  public static final double odometryFrequency = 250.0; // Hz (100 for CAN, 250 for CANFD)
  public static final double trackWidth = Units.inchesToMeters(26.5);
  public static final double wheelBase = Units.inchesToMeters(26.5);
  public static final double driveBaseRadius = Math.hypot(wheelBase / 2.0, wheelBase / 2.0);
  public static final double driveWheelRadiusMeters = Units.inchesToMeters(2);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  public static final double kSteerInertia = 0.004;
  public static final double kDriveInertia = 0.025;

  public static final LinearVelocity maxSpeedAt12Volts =
      FeetPerSecond.of(15); // MK4i 16.5 ft/s L3 Kraken FOC With 14t pinion

  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double driveMotorGearRatio =
      1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0)); // Mk4i L3 with 14t pinion
  public static final DCMotor driveGearbox = DCMotor.getKrakenX60Foc(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / driveMotorGearRatio; // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / driveMotorGearRatio; // Rotor RPM -> Wheel Rad/Sec

  // Turn motor configuration
  public static final double steerMotorGearRatio = 150.0 / 7.0; // MK4i
  public static final int turnMotorCurrentLimit = 20;
  public static final DCMotor turnGearbox = DCMotor.getKrakenX60Foc(1);

  // Turn encoder configuration
  public static final double turnEncoderPositionFactor =
      2 * Math.PI / steerMotorGearRatio; // Rotations -> Radians
  public static final double turnEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / steerMotorGearRatio; // RPM -> Rad/Sec

  // PathPlanner configuration
  public static final double robotMassKg = 74.088;
  public static final double robotMOI = 6.883;
  public static final double wheelCOF = 1.2;
  public static final RobotConfig ppConfig =
      new RobotConfig(
          robotMassKg,
          robotMOI,
          new ModuleConfig(
              driveWheelRadiusMeters,
              maxSpeedAt12Volts.in(MetersPerSecond),
              wheelCOF,
              driveGearbox.withReduction(driveMotorGearRatio),
              driveMotorCurrentLimit,
              1),
          moduleTranslations);

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withCustomModuleTranslations(moduleTranslations)
          .withRobotMass(Kilogram.of(robotMassKg))
          .withGyro(COTS.ofNav2X())
          .withSwerveModule(
              () ->
                  new SwerveModuleSimulation(
                      new SwerveModuleSimulationConfig(
                          driveGearbox,
                          turnGearbox,
                          driveMotorGearRatio,
                          18.0,
                          Volts.of(0.1),
                          Volts.of(0.1),
                          Meters.of(driveWheelRadiusMeters),
                          KilogramSquareMeters.of(0.02),
                          wheelCOF)));
}
