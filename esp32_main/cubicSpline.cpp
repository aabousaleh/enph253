#include "cubicSpline.h"

void computeCubicSplineCoefficients(Point *points, Spline *splines, int n, char coordinate) {
    int i;
    double h[n-1], alpha[n-1], l[n], mu[n], z[n];
    double a[n], b[n], c[n], d[n];

    // Choose the coordinate to interpolate
    for (i = 0; i < n; i++) {
        if (coordinate == 'x') {
            a[i] = points[i].x;
        } else {
            a[i] = points[i].y;
        }
    }

    for (i = 0; i < n-1; i++) {
        h[i] = points[i+1].time - points[i].time;
    }

    for (i = 1; i < n-1; i++) {
        alpha[i] = (3.0/h[i]) * (a[i+1] - a[i]) - (3.0/h[i-1]) * (a[i] - a[i-1]);
    }

    l[0] = 1.0;
    mu[0] = z[0] = 0.0;

    for (i = 1; i < n-1; i++) {
        l[i] = 2.0 * (points[i+1].time - points[i-1].time) - h[i-1] * mu[i-1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i-1] * z[i-1]) / l[i];
    }

    l[n-1] = 1.0;
    z[n-1] = c[n-1] = 0.0;

    for (i = n-2; i >= 0; i--) {
        c[i] = z[i] - mu[i] * c[i+1];
        b[i] = (a[i+1] - a[i]) / h[i] - h[i] * (c[i+1] + 2.0 * c[i]) / 3.0;
        d[i] = (c[i+1] - c[i]) / (3.0 * h[i]);
    }

    for (i = 0; i < n-1; i++) {
        splines[i].a = a[i];
        splines[i].b = b[i];
        splines[i].c = c[i];
        splines[i].d = d[i];
        splines[i].t = points[i].time;
    }
}

double evaluateSpline(Spline *splines, int n, double time) {
        int i;
    for (i = 0; i < n-1; i++) {
        if (time >= splines[i].t && time <= splines[i+1].t) {
            double dt = time - splines[i].t;
            return splines[i].a + splines[i].b * dt + splines[i].c * dt * dt + splines[i].d * dt * dt * dt;
        }
    }
    // If the time is beyond the last point, return the value at the last segment
    if (time >= splines[n-2].t) {
        double dt = time - splines[n-2].t;
        return splines[n-2].a + splines[n-2].b * dt + splines[n-2].c * dt * dt + splines[n-2].d * dt * dt * dt;
    }
    // If the time is before the first point, return the value at the first segment
    if (time < splines[0].t) {
        return splines[0].a;
    }
    return 0.0;
}