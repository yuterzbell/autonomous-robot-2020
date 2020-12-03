package ca.mcgill.ecse211.test;

import static ca.mcgill.ecse211.project.Navigation.*;
import static ca.mcgill.ecse211.project.Resources.*;
import static org.junit.jupiter.api.Assertions.*;

import ca.mcgill.ecse211.playingfield.Point;
import org.junit.jupiter.api.Test;

/**
 * Tests the Navigation class. This test runs in Eclipse (right-click > Run as > Unit test) and
 * on the command line, not in Webots!
 * 
 * @author Younes Boubekeur
 */
public class TestNavigation {
  /** Tolerate up to this amount of error due to double imprecision. */
  private static final double ERROR_MARGIN = 0.01;
  
  @Test void testMinimalAngle() {
    // Going from 45° to 135° means turning by +90°
    assertEquals(90, minimalAngle(45, 135), ERROR_MARGIN);
    
    // Going from 185° to 175° means turning by -10°
    assertEquals(-10, minimalAngle(185, 175), ERROR_MARGIN);
    
    // Going from 90° to 90° means turning by 0°
    assertEquals(0, minimalAngle(90, 90), ERROR_MARGIN);
    
    // TODO Add more test cases here. Don't forget about edge cases!
    // Going from 0° to 360° means do not turn
    assertEquals(0, minimalAngle(0, 360), ERROR_MARGIN);
    
    // Going from -10° to 330° means turning by -20°
    assertEquals(-20, minimalAngle(-10, 330), ERROR_MARGIN);
    
    // Going from 330° to -10° means turning by 20°
    assertEquals(20, minimalAngle(330, -10), ERROR_MARGIN);
    
    // Going from 0° to 180° means turning by 180° -- edge case
    assertEquals(180, minimalAngle(0, 180), ERROR_MARGIN);
    
    // Going from 180° to 360° means turning by 180°
    assertEquals(180, minimalAngle(180, 360), ERROR_MARGIN);
    
    // Going from 90° to 270° means turning by 180°
    assertEquals(180, minimalAngle(90, 270), ERROR_MARGIN);
    
    // Going from 270° to 90° means turning by -180°
    assertEquals(-180, minimalAngle(270, 90), ERROR_MARGIN);
    
    
  }
  
 
  
  
  @Test void testgetDestinationAngle() {
    
    // Heading to 0°
    assertEquals(0, getDestinationAngle(new Point(0, 0), new Point(0, 1)), ERROR_MARGIN);
    
    // Heading to 45°
    assertEquals(45, getDestinationAngle(new Point(0, 0), new Point(1, 1)), ERROR_MARGIN);
    
    // Heading to 36.87°
    assertEquals(36.87, getDestinationAngle(new Point(0, 0), new Point(3, 4)), ERROR_MARGIN);
    
    // Heading be 90°
    assertEquals(90, getDestinationAngle(new Point(0, 0), new Point(1, 0)), ERROR_MARGIN);
    
    // when destination is to the right-bottom of current position
    
    // Heading be 123.69°
    assertEquals(123.69, getDestinationAngle(new Point(0, 0), new Point(3, -2)), ERROR_MARGIN);
    
    // Heading be 180°
    assertEquals(180, getDestinationAngle(new Point(0, 0), new Point(0, -1)), ERROR_MARGIN);
    
    // when destination is to the left-bottom of current position
   
    // Heading be 236.31°
    assertEquals(236.31, getDestinationAngle(new Point(0, 0), new Point(-3, -2)), ERROR_MARGIN);
    
    // Heading be 270°
    assertEquals(270, getDestinationAngle(new Point(0, 0), new Point(-1, 0)), ERROR_MARGIN);
    
    // when destination is to the left-top of current position
    
    // Heading be 303.69°
    assertEquals(303.69, getDestinationAngle(new Point(0, 0), new Point(-3, 2)), ERROR_MARGIN);
    
    // Heading be 315°
    assertEquals(315, getDestinationAngle(new Point(0, 0), new Point(-1, 1)), ERROR_MARGIN);
    
    
  }
  
  
  @Test void testdistanceBetween() {
    assertEquals(1, distanceBetween(new Point(0, 0), new Point(0, 1)), ERROR_MARGIN);
    
    assertEquals(1.41, distanceBetween(new Point(0, 0), new Point(1, 1)), ERROR_MARGIN);
    
    assertEquals(4.12, distanceBetween(new Point(1, 3), new Point(5, 2)), ERROR_MARGIN);
    
    assertEquals(3.60, distanceBetween(new Point(-3, -2), new Point(0, 0)), ERROR_MARGIN);
    
    assertEquals(5.0, distanceBetween(new Point(-3, -2), new Point(1, -5)), ERROR_MARGIN);
    
    assertEquals(3.16, distanceBetween(new Point(5, -2), new Point(6, -5)), ERROR_MARGIN);
    
    assertEquals(4.12, distanceBetween(new Point(-2, 6), new Point(6, -5)), ERROR_MARGIN);
    
    assertEquals(13.60, distanceBetween(new Point(-2, 6), new Point(6, -5)), ERROR_MARGIN);
    
    assertEquals(4.12, distanceBetween(new Point(-2, -6), new Point(-6, -5)), ERROR_MARGIN);
    
    assertEquals(8.06, distanceBetween(new Point(2, -6), new Point(-6, -5)), ERROR_MARGIN);
    
    assertEquals(4.47, distanceBetween(new Point(-3, -2), new Point(1, -5)), ERROR_MARGIN);
  
  }
  
  


}
