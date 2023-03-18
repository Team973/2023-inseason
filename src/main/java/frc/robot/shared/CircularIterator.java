package frc.robot.shared;

import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;

public class CircularIterator<T> implements Iterator<T> {
  private List<T> m_list;
  private ListIterator<T> m_iterator;

  public CircularIterator(List<T> list) {
    if (list.isEmpty()) {
      throw new IllegalArgumentException("Elements list cannot be empty");
    }
    m_list = list;
    m_iterator = list.listIterator();
  }

  @Override
  public boolean hasNext() {
    return true;
  }

  public boolean hasPrevious() {
    return true;
  }

  @Override
  public T next() {
    if (!m_iterator.hasNext()) {
      m_iterator = m_list.listIterator();
    }
    return m_iterator.next();
  }

  public T previous() {
    if (!m_iterator.hasPrevious()) {
      m_iterator = m_list.listIterator(m_list.size());
    }
    return m_iterator.previous();
  }
}
