#include <Arduino.h>
#include "arm.h"
#include "cubicSpline.h"

Point points[] = {//x,y,t
        {40, 9.0, 0.0},
        {35.2, 23.4, 1000.0},
        {20, 12.4, 2000.0},
        {40, 9.0, 3000.0},
        {35.2, 23.4, 4000.0}
    };

const int n = sizeof(points) / sizeof(points[0]);

Spline x_splines[n-1];
Spline y_splines[n-1];

void setup() {
 setupArmServos();
 
 // Compute spline coefficients for x and y
 computeCubicSplineCoefficients(points, x_splines, n, 'x');
 computeCubicSplineCoefficients(points, y_splines, n, 'y');
}

void loop() {
  double x = evaluateSpline(x_splines, n, millis() % 3000);
  double y = evaluateSpline(y_splines, n, millis() % 3000);
  moveToXY(x,y);
  delay(10);
}