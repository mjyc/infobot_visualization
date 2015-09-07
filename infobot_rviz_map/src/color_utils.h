#ifndef INFOBOT_RVIZ_MAP_COLOR_UTILS_H
#define INFOBOT_RVIZ_MAP_COLOR_UTILS_H

// MATLAB Jet Color Palette
// From http://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
// r, g, b are in [0.0,1.0]
inline void colorMATLABJetPalette(double v, double vmin, double vmax, double& r, double& g, double& b)
{
  r = 1.0;
  g = 1.0;
  b = 1.0;

  double dv;

  if (v < vmin)
    v = vmin;
  if (v > vmax)
    v = vmax;
  dv = vmax - vmin;

  if (v < (vmin + 0.25 * dv))
  {
    r = 0;
    g = 4 * (v - vmin) / dv;
  }
  else if (v < (vmin + 0.5 * dv))
  {
    r = 0;
    b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
  }
  else if (v < (vmin + 0.75 * dv))
  {
    r = 4 * (v - vmin - 0.5 * dv) / dv;
    b = 0;
  }
  else
  {
    g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
    b = 0;
  }
}

#endif  // INFOBOT_RVIZ_MAP_COLOR_UTILS_H
