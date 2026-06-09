#ifndef SWARM_MATH_UTILS_H
#define SWARM_MATH_UTILS_H

#include "types.h"

double wrap_pi(double angle);
double clamp(double value, double lo, double hi);

void flat_earth_to_lla(double x_n, double y_e, double lon0, double lat0,
                       double *lon_out, double *lat_out);
void lla_to_flat_earth(double lon, double lat, double lon0, double lat0,
                       double *x_out, double *y_out);

double bearing_between(double lat1, double lon1, double lat2, double lon2);
double distance_between(double lat1, double lon1, double lat2, double lon2);

double cross_track_distance(double lat_now, double lon_now,
                            double lat_start, double lon_start,
                            double lat_end, double lon_end);

void offset_position(double distance_km, double angle_rad,
                     double lon_rad, double lat_rad,
                     double *lon_out, double *lat_out);

double f_dyn(double x[], int i, double nx, double nz, double phi);
void runge4(double x[], double step, double nx, double nz, double phi);

double sign_val(double x);

#endif
