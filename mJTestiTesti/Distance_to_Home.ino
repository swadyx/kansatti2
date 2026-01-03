#include <math.h>

double deg_to_rad(double deg) {
  return deg * (PI / 180.0);
}

double distance_to_home(double current_lat, double current_lon, double HOME_LAT, double HOME_LON) {
  const double EARTH_RADIUS_KM = 6365.0;
  double dLat = deg_to_rad(HOME_LAT - current_lat);
  double dLon = deg_to_rad(HOME_LON - current_lon);

  double a =
    sin(dLat / 2) * sin(dLat / 2) +
    cos(deg_to_rad(current_lat)) *
    cos(deg_to_rad(HOME_LAT)) *
    sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));

  return EARTH_RADIUS_KM * c;  // kilometers
}
//homecoordinates NEEDS to be setupped before flight! Also can make function to get homecoordinates in preflight-state and use them. (coordinates on SD?)
