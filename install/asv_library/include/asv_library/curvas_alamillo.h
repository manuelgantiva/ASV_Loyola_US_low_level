#ifndef CURVAS_ALAMILLO_H
#define CURVAS_ALAMILLO_H


struct Target {
    double xp;
    double yp;
    double dxp;
    double dyp;
    double phip;
    double dphip;
    double f_c;
};

void recta(double w, Target* px, double ax, double by, double mx, double my);
void arco(double w, Target* px, double ax, double by, double r, bool sig) ;

//Circuito 1
// Closed semicircle radius 2, straight line 30
Target curva_ala_1_2(double w);
// Closed semicircle radius 4, straight line 30
Target curva_ala_1_4(double w);
// Closed semicircle radius 6, straight line 30
Target curva_ala_1_6(double w);

//Trayectoria Paper
Target line_south(double w);
Target circle_10m(double w);
Target circle_8m(double w);
Target lissajous_10m(double w);
Target lissajous_5m(double w);

#endif