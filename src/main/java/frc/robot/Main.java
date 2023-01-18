// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileInputStream;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.apache.logging.log4j.core.config.ConfigurationSource;
import org.apache.logging.log4j.core.config.Configurator;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot
 * call.
 */
public final class Main {
  private static final Logger logger = LogManager.getLogger();

  private Main() {
    // Configure Log4j
    try {
      final ConfigurationSource source =
          new ConfigurationSource(
              new FileInputStream(Filesystem.getDeployDirectory() + "/log4j2.xml"));
      Configurator.initialize(null, source);
    } catch (IOException e) {
      System.err.println(e);
      System.exit(1);
    }
    Thread.setDefaultUncaughtExceptionHandler(new GlobalExceptionHandler());

    logger.info("This is a test message");
  }

  /**
   * Main initialization function. Do not perform any initialization here.
   *
   * <p>If you change your main robot class, change the parameter type.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
