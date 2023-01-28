package frc.robot.greydash;

import java.io.IOException;
import java.util.Date;
import java.util.HashMap;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import lombok.Getter;

/** A chart that can be displayed on the GreyDash dashboard. */
public class GreyDashChart {
  @Getter private final String name;
  @Getter private final HashMap<String, Series> seriesMap;
  private int numIdx = 0;

  private final StringPublisher publisher;

  /**
   * Creates a new chart with the given name. You probably want to use the static method in
   * GreyDashClient instead.
   *
   * @param chartsTable The NetworkTable that contains the charts.
   * @param name The name of the chart.
   */
  public GreyDashChart(final NetworkTable chartsTable, final String name) {
    this.publisher = chartsTable.getStringTopic(name).publish();
    seriesMap = new HashMap<>();
    this.name = name;
  }

  /**
   * Adds a data point to a given series on the chart.
   *
   * @param label The label of the data series.
   * @param value The value of the data point.
   */
  public void addDataToSeries(final String label, final double value) {
    if (!seriesMap.containsKey(label)) {
      seriesMap.put(label, new Series(numIdx));
      numIdx++;
    }
    Series series = seriesMap.get(label);
    series.updateValue(value, new Date());

    updateNT();
  }

  /** A data series on a chart. */
  private class Series {
    @Getter private final int index;
    @Getter private Double latestValue;
    @Getter private Date timestamp;

    /**
     * Creates a new data series.
     *
     * @param index The index of the series.
     */
    public Series(int index) {
      this.index = index;
    }

    /**
     * Updates the current value of the series.
     *
     * @param value The new value.
     * @param timestamp The timestamp of the value.
     */
    public void updateValue(double value, Date timestamp) {
      this.latestValue = value;
      this.timestamp = timestamp;
    }
  }

  /**
   * Converts the chart to a JSON string.
   *
   * @return The JSON string.
   * @throws IOException If the JSON string cannot be created.
   */
  public String toJson() throws IOException {
    ObjectMapper mapper = new ObjectMapper();
    return mapper.writeValueAsString(this);
  }

  /** Updates the NetworkTable with the current chart data. */
  public void updateNT() {
    try {
      publisher.set(toJson());
    } catch (IOException e) {
      System.err.println(e);
    }
  }
}
