package frc.robot.greydash;

import java.io.File;
import java.io.IOException;
import java.net.InetSocketAddress;

import frc.robot.shared.SimpleWebServer;

import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.wpilibj.Filesystem;
import lombok.AllArgsConstructor;

@AllArgsConstructor
public class GreyDashServer extends Thread {
  private final int m_port;

  public void run() {
    try {
      HttpServer server = HttpServer.create(new InetSocketAddress(m_port), 0);
      SimpleWebServer.create(
          server,
          "/",
          (new File(Filesystem.getDeployDirectory(), "greydash")).getAbsolutePath(),
          "index.html");
      server.start();
    } catch (IOException e) {
      System.err.println("Web Server error: " + e);
    }
  }
}
