#ifndef CUBICSPLINE_H
#define CUBICSPLINE_H
/**
 * This library does cubic spline interpolation
 */

// Structure for storing a point
typedef struct {
    double x;
    double y;
    double time;
} Point;

// Structure for storing the coefficients of the cubic spline
typedef struct {
    double a;
    double b;
    double c;
    double d;
    double t;
} Spline;

//Takes a point and sets the coefficients of the spline object.
void computeCubicSplineCoefficients(Point *points, Spline *splines, int n, char coordinate);

double evaluateSpline(Spline *splines, int n, double time);

#endif
