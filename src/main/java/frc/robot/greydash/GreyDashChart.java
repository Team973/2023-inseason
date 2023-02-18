package frc.robot.greydash;

import java.io.IOException;
import java.util.Date;
import java.util.HashMap;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.StringPublisher;
import lombok.Getter;
import lombok.experimental.Accessors;

/** A chart that can be displayed on the GreyDash dashboard. */
@Accessors(prefix = "m_")
public class GreyDashChart {
  @Getter private final String m_name;
  @Getter private final HashMap<String, Series> m_seriesMap;
  private int m_numIdx = 0;

  private final StringPublisher m_publisher;

  /**
   * Creates a new chart with the given name. You probably want to use the static method in
   * GreyDashClient instead.
   *
   * @param chartsTable The NetworkTable that contains the charts.
   * @param name The name of the chart.
   */
  public GreyDashChart(final NetworkTable chartsTable, final String name) {
    m_publisher = chartsTable.getStringTopic(name).publish();
    m_seriesMap = new HashMap<>();
    m_name = name;
  }

  /**
   * Adds a data point to a given series on the chart.
   *
   * @param label The label of the data series.
   * @param value The value of the data point.
   */
  public void addDataToSeries(final String label, final double value) {
    if (!m_seriesMap.containsKey(label)) {
      m_seriesMap.put(label, new Series(m_numIdx));
      m_numIdx++;
    }
    Series series = m_seriesMap.get(label);
    series.updateValue(value, new Date());

    updateNT();
  }

  /** A data series on a chart. */
  private class Series {
    @Getter private final int m_index;
    @Getter private Double m_latestValue;
    @Getter private Date m_timestamp;

    /**
     * Creates a new data series.
     *
     * @param index The index of the series.
     */
    public Series(int index) {
      m_index = index;
    }

    /**
     * Updates the current value of the series.
     *
     * @param value The new value.
     * @param timestamp The timestamp of the value.
     */
    public void updateValue(double value, Date timestamp) {
      m_latestValue = value;
      m_timestamp = timestamp;
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
      m_publisher.set(toJson());
    } catch (IOException e) {
      System.err.println(e);
    }
  }
}
