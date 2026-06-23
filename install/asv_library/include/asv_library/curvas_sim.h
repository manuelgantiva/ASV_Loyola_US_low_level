#ifndef CURVAS_SIM_H
#define CURVAS_SIM_H


struct Target {   // Declare target struct type
    float xp;  // Declare member types
    float yp;
    float dxp;
    float dyp;
    float phip;
    float dphip;
    float f_c;
};

void recta(double w, Target* px, double ax, double by, double mx, double my);
void arco(double w, Target* px, double ax, double by, double r, bool sig);

Target circle_30m(double w);
Target line_northeast(double w);

#endif