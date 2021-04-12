#include <stdio.h>
#include <stdlib.h>
#include <math.h>


/* Some <math.h> files do not define M_PI... */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
  double x, y;
} Point2;

/****************************************************************************
 * 
 * http://www.mit.edu/course/6/6.837/src/glut-3.7/progs/examples/circlefit.c
 * 
 ****************************************************************************/

int CircleFit(int N, Point2 * P, double *pa, double *pb, double *pr)
{
  /* user-selected parameters */
  const int maxIterations = 256;
  const double tolerance = 1e-06;

  double a, b, r;

  /* compute the average of the data points */
  int i, j;
  double xAvr = 0.0;
  double yAvr = 0.0;

  for (i = 0; i < N; i++) {
    xAvr += P[i].x;
    yAvr += P[i].y;
  }
  xAvr /= N;
  yAvr /= N;

  /* initial guess */
  a = xAvr;
  b = yAvr;

  for (j = 0; j < maxIterations; j++) {
    /* update the iterates */
    double a0 = a;
    double b0 = b;

    /* compute average L, dL/da, dL/db */
    double LAvr = 0.0;
    double LaAvr = 0.0;
    double LbAvr = 0.0;

    for (i = 0; i < N; i++) {
      double dx = P[i].x - a;
      double dy = P[i].y - b;
      double L = sqrt(dx * dx + dy * dy);
      if (fabs(L) > tolerance) {
        LAvr += L;
        LaAvr -= dx / L;
        LbAvr -= dy / L;
      }
    }
    LAvr /= N;
    LaAvr /= N;
    LbAvr /= N;

    a = xAvr + LAvr * LaAvr;
    b = yAvr + LAvr * LbAvr;
    r = LAvr;

    if (fabs(a - a0) <= tolerance && fabs(b - b0) <= tolerance)
      break;
  }

  *pa = a;
  *pb = b;
  *pr = r;

  return (j < maxIterations ? j : -1);
}

enum {
  M_SHOW_CIRCLE, M_CIRCLE_INFO, M_RESET_POINTS, M_QUIT
};

#define MAX_POINTS 100

int num = 0;
Point2 list[MAX_POINTS];
int circleFitNeedsRecalc = 0;

int circleInfo = 0;

double a, b, r = 0.0;   /* X, Y, and radius of best fit circle.      */

void
addPoint(double x, double y)
{
  if (num + 1 >= MAX_POINTS) {
    fprintf(stderr, "circlefit: limited to only %d points\n", MAX_POINTS);
    return;
  }
  list[num].x = x;
  list[num].y = y;
  num++;
  circleFitNeedsRecalc = 1;
   
}
                     
void setup()
{
  addPoint(10,7);
  addPoint(3,5.5);
  addPoint(27,3);
}

void loop()
{
  int i;

  if (circleFitNeedsRecalc) {
    int rc;

    rc = CircleFit(num, list, &a, &b, &r);
    if (rc == -1) {
      fprintf(stderr, "circlefit: Problem fitting points to a circle encountered.\n");
    } else {
      if (circleInfo) {
        printf("%g @ (%g,%g)\n", r, a, b);
      }
    }
    circleFitNeedsRecalc = 0;
  }

  for (i = 0; i < num; i++) {
  //  glVertex2d(list[i].x, list[i].y);
  }
 // glEnd();
 // glutSwapBuffers();
}
