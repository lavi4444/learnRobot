// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;
import java.util.Set;

import org.opencv.dnn.Model;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;

public class RobotContainer {
  
  private final CommandPS5Controller controller = new CommandPS5Controller(0);
  private SwerveModuleSubsystem module = new SwerveModuleSubsystem();
  
  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {
    controller.cross().whileTrue(Commands.startEnd(() -> {
      module.setPowerAngle(0.3);
    }, () -> {
      module.stop();
    }, module));

    controller.triangle().whileTrue(Commands.startEnd(() -> {
      module.setPowerAngle(-0.3);
    }, () -> {
      module.stop();
    }, module));

    module.setDefaultCommand(Commands.run(() -> {
      double deadbandedSpeed = MathUtil.applyDeadband(controller.getLeftX(), 0.03);
      double deadbandedSpeedDrive = MathUtil.applyDeadband(controller.getLeftY(), 0.03);
      module.setPowerAngle(deadbandedSpeed);
      module.setPowerDrive(deadbandedSpeedDrive);
    }, module));

    controller.square().onTrue(Commands.run( () -> {
      if (module.getAngle().getDegrees() > 90) {
        module.setPowerAngle(-0.3);
      }
      else {
        module.setPowerAngle(0.3);
      }
    }, module).until( () -> {
      return module.getAngle().getDegrees() > 89 && module.getAngle().getDegrees() < 91;
    }).finallyDo( () -> {
      module.stop();
    } ));

    controller.R3().onTrue(Commands.sequence(
      Commands.run( () -> {
      if (module.getAngle().getDegrees() > 90) {
        module.setPowerAngle(-0.3);
      }
      else {
        module.setPowerAngle(0.3);
      }
      }, module).until( () -> {
        return module.getAngle().getDegrees() > 89 && module.getAngle().getDegrees() < 91;
      }).finallyDo( () -> {
        module.stop();
      } ),

      Commands.runOnce( () -> {
        module.setPowerDrive(0.4);
      }, module), 

      Commands.waitSeconds(1),

      Commands.runOnce( () -> {
        module.stopDrive();
      }, module)
      ));

    controller.circle().onTrue(Commands.sequence(
      Commands.run( () -> {
      if (module.getAngle().getDegrees() > 90) {
        module.setPowerAngle(-0.3);
      }
      else {
        module.setPowerAngle(0.3);
      }
    }, module).until( () -> {
      return module.getAngle().getDegrees() > 89 && module.getAngle().getDegrees() < 91;
    }).finallyDo( () -> {
      module.stop();
    } ),
    Commands.waitSeconds(1),
    
    Commands.run( () -> {
      if (module.getAngle().getDegrees() > 0) {
        module.setPowerAngle(-0.3);
      }
      else {
        module.setPowerAngle(0.3);
      }
    }, module).until( () -> {
      return module.getAngle().getDegrees() > -1 && module.getAngle().getDegrees() < 1;
    }).finallyDo( () -> {
      module.stop();
    } )
    ));

    controller.R2().whileTrue(Commands.runEnd(() -> {
      module.setPowerDrive(0.4);
    }, () -> {
      module.stopDrive();
    }, module));




    //5
    //a
    controller.L3().onTrue(Commands.sequence(
      module.goToState(0, 90).until(() -> {
      return module.atAngle();
    }).withTimeout(5), 
    Commands.waitSeconds(1),
      module.goToState(0, 0).until(() -> {
      return module.atAngle();
    }).withTimeout(5)
    ));
    //b
    controller.R3().onTrue(module.goToState(0.6, 45).until(() -> {
      return module.atAngle();
    }).withTimeout(5));
    //c





    //2
    controller.circle().whileTrue(module.goToState(0.7, 78));

    //שאלה 4
    controller.povLeft().whileTrue(Commands.sequence(
      Commands.runOnce(() -> {
        module.setPowerDrive(0.4);
      }, module),

      Commands.waitSeconds(1.5),

      Commands.runOnce(() -> {
        module.stopDrive();
      }, module),

      Commands.waitSeconds(0.5),

      Commands.runOnce(() -> {
        module.setPowerDrive(-1.5);
      }, module),

      Commands.waitSeconds(1.5),

      Commands.runOnce(() -> {
        module.stopDrive();
      }, module)
    ).finallyDo(() -> {
      module.stopDrive();
    }));

    controller.povDown().onTrue(Commands.sequence(

    Commands.run(() -> {
      module.setPowerDrive(0.1);
    }, module),

    module.rotate(Rotation2d.fromDegrees(45)),

    Commands.runOnce(() -> {
      module.stopDrive();
    }, module)
    ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}

