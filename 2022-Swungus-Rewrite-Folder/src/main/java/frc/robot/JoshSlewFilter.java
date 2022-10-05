// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class JoshSlewFilter {
  private final double m_rateLimitForward;
  private final double m_rateLimitReverse;
  private final double m_maxValueLimitForward;
  private final double m_maxValueLimitReverse;
  private final double m_fullOutputForward;
  private final double m_fullOutputReverse;
  private final double m_limitOutputForward;
  private final double m_limitOutputReverse;
  private double m_prevVal;
  private double m_prevTime;

  private double m_modifiedRateLimitForward;
  private double m_modifiedRateLimitReverse;

  /**
   * Creates a new SlewRateLimiter with the given rate limit and initial value.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   * @param initialValue The initial value of the input.
   */
  public JoshSlewFilter(double rateLimitForward, double rateLimitReverse, double initialValue, double maxValueLimitForward, double maxValueLimitReverse, double fullOutputForward, double fullOutputReverse, double limitOutputForward, double limitOutputReverse) {
    m_rateLimitForward = rateLimitForward;
    m_rateLimitReverse = rateLimitReverse;
    m_maxValueLimitForward = maxValueLimitForward;
    m_maxValueLimitReverse = maxValueLimitReverse;
    m_fullOutputForward = fullOutputForward;
    m_fullOutputReverse = fullOutputReverse;
    m_limitOutputReverse = limitOutputReverse;
    m_limitOutputForward = limitOutputForward;
    m_prevVal = initialValue;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }

  /**
   * Creates a new SlewRateLimiter with the given rate limit and an initial value of zero.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public JoshSlewFilter(double rateLimit) {
    this(rateLimit, rateLimit, 0, rateLimit, rateLimit, 1, 1, 1, -1);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    //calculate slews
    m_modifiedRateLimitReverse = m_rateLimitReverse-Math.abs((m_rateLimitReverse-m_maxValueLimitReverse)*(m_prevVal/m_fullOutputReverse));
    m_modifiedRateLimitForward = m_rateLimitForward-Math.abs((m_rateLimitForward-m_maxValueLimitForward)*(m_prevVal/m_fullOutputForward));

    //rate limit
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    m_prevVal +=
        MathUtil.clamp(input - m_prevVal, -m_modifiedRateLimitReverse * elapsedTime, m_modifiedRateLimitForward * elapsedTime);
    m_prevVal = MathUtil.clamp(m_prevVal, m_limitOutputReverse, m_limitOutputForward);
    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }
}