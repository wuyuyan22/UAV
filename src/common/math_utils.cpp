#include "common/math_utils.h"
#include <cmath>

double wrap_pi(double angle) {
    while (angle > SWARM_PI)  angle -= 2.0 * SWARM_PI;
    while (angle < -SWARM_PI) angle += 2.0 * SWARM_PI;
    return angle;
}

double clamp(double value, double lo, double hi) {
    return (value > hi) ? hi : ((value < lo) ? lo : value);
}

void flat_earth_to_lla(double x_n, double y_e, double lon0, double lat0,
                       double *lon_out, double *lat_out) {
    double dLat = x_n / SWARM_RE;
    double dLon = y_e / (SWARM_RE * cos(lat0));
    *lon_out = wrap_pi(lon0 + dLon);
    *lat_out = clamp(lat0 + dLat, -SWARM_PI / 2, SWARM_PI / 2);
}

void lla_to_flat_earth(double lon, double lat, double lon0, double lat0,
                       double *x_out, double *y_out) {
    *x_out = (lat - lat0) * SWARM_RE;
    *y_out = (lon - lon0) * SWARM_RE * cos(lat0);
}

double bearing_between(double lat1, double lon1, double lat2, double lon2) {
    double dLon = lon2 - lon1;
    double theta = atan2(sin(dLon) * cos(lat2),
                         cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon));
    return wrap_pi(theta);
}

double distance_between(double lat1, double lon1, double lat2, double lon2) {
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a = sin(dLat * 0.5) * sin(dLat * 0.5) +
               sin(dLon * 0.5) * sin(dLon * 0.5) * cos(lat1) * cos(lat2);
    return SWARM_RE * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

double cross_track_distance(double lat_now, double lon_now,
                            double lat_start, double lon_start,
                            double lat_end, double lon_end) {
    double dist = distance_between(lat_now, lon_now, lat_end, lon_end);
    if (dist < 0.1) return 0;
    double brng_end   = bearing_between(lat_now, lon_now, lat_end, lon_end);
    double brng_track = bearing_between(lat_start, lon_start, lat_end, lon_end);
    return dist * sin(wrap_pi(brng_track - brng_end));
}

void offset_position(double distance_km, double angle_rad,
                     double lon_rad, double lat_rad,
                     double *lon_out, double *lat_out) {
    double arc = distance_km * 1.56960994449e-04;
    *lat_out = asin(sin(lat_rad) * cos(arc) + cos(lat_rad) * sin(arc) * cos(angle_rad));
    *lon_out = lon_rad + asin(sin(angle_rad) * sin(arc) / cos(*lat_out));
}

double f_dyn(double x[], int i, double nx, double nz, double phi) {
    switch (i) {
    case 0: return SWARM_GRAVITY * (nx - sin(x[1]));
    case 1: return SWARM_GRAVITY * (nz * cos(phi) - cos(x[1])) / x[0];
    case 2: return SWARM_GRAVITY * fabs(nz) * sin(phi) / (x[0] * cos(x[1]));
    case 3: return x[0] * cos(x[1]) * cos(x[2]);
    case 4: return x[0] * cos(x[1]) * sin(x[2]);
    case 5: return x[0] * sin(x[1]);
    default: return 0;
    }
}

void runge4(double x[], double step, double nx, double nz, double phi) {
    double k1[6], k2[6], k3[6], k4[6], xn[6];

    for (int j = 0; j < 6; j++) xn[j] = x[j];
    for (int i = 0; i < 6; i++) k1[i] = f_dyn(x, i, nx, nz, phi);

    for (int j = 0; j < 6; j++) xn[j] = x[j] + 0.5 * step * k1[j];
    for (int i = 0; i < 6; i++) k2[i] = f_dyn(xn, i, nx, nz, phi);

    for (int j = 0; j < 6; j++) xn[j] = x[j] + 0.5 * step * k2[j];
    for (int i = 0; i < 6; i++) k3[i] = f_dyn(xn, i, nx, nz, phi);

    for (int j = 0; j < 6; j++) xn[j] = x[j] + step * k3[j];
    for (int i = 0; i < 6; i++) k4[i] = f_dyn(xn, i, nx, nz, phi);

    for (int i = 0; i < 6; i++)
        x[i] += step * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]) / 6.0;

    x[1] = wrap_pi(x[1]);
    x[2] = wrap_pi(x[2]);
}

double sign_val(double x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}
