// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.system.DCMotor;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;

/** This is a seperate class because MapleSim may not fully support the Rebuilt game on release */
public class DriveMapleSim extends Drive {
    private final SelfControlledSwerveDriveSimulation simulation;
    @SuppressWarnings("unchecked")
	public DriveMapleSim(
        GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
        super(gyroIO, flModuleIO, frModuleIO, blModuleIO, brModuleIO);
        if (Robot.isReal()) {
            throw new RuntimeException("This does not and will not support running on a real robot please change DriveMapleSim to Drive in the code please");
        }
        simulation = new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(
                new DriveTrainSimulationConfig(Pounds.of(115), Inches.of(32), Inches.of(32), Meters.of(TunerConstants.FrontLeft.LocationX).times(2.0), Meters.of(TunerConstants.FrontLeft.LocationY).times(2.0), COTS.ofPigeon2(), COTS.ofSwerveX2S(DCMotor.getKrakenX60(1), DCMotor.getKrakenX44(1), COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof, 2, 18)), 
                Constants.initialPose));
        SimulatedArena.getInstance().addDriveTrainSimulation(simulation.getDriveTrainSimulation());
    }

    @Override
    public Pose2d getPose() {
        if (this.simulation != null) {
            return Constants.useEstimated ? simulation.getOdometryEstimatedPose() : simulation.getActualPoseInSimulationWorld();
        }
        return Constants.initialPose;
    }

    @Override
    public void periodic() {
        super.periodic();
        simulation.runSwerveStates(getModuleStates());
    }
}
