double DISTANCE_BETWEEN(double lat1, double lon1, double lat2, double lon2) {
  const double Radius_Earth = 6365.0;
  double dLat = deg_to_rad(lat2 - lat1);
  double dLon = deg_to_rad(lon2 - lon1);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(deg_to_rad(lat1)) * cos(deg_to_rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = Radius_Earth * c;
  return distance;
}


double deg_to_rad(double deg) {
  return deg * (3.141592653589793 / 180.0);
}
