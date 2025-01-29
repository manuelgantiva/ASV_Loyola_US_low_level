#ifndef CURVAS_ALAMILLO_H
#define CURVAS_ALAMILLO_H


struct Target {   // Declare target struct type
    float xp;  // Declare member types
    float yp;
    float dxp;
    float dyp;
    float f_c;
};

void recta(float w, Target* px, float ax, float by, float mx, float my);
void arco(float w, Target* px, float ax, float by, float r, bool sig) ;

//Circuito 1
// Closed semicircle radius 2, straight line 30
Target curva_ala_1_2(float w);
// Closed semicircle radius 4, straight line 30
Target curva_ala_1_4(float w);
// Closed semicircle radius 6, straight line 30
Target curva_ala_1_6(float w);

//Circuito 2
// Closed quarter radius 6, straight line 50
Target curva_ala_2_2(float w);
// Closed quarter radius 8, straight line 50
Target curva_ala_2_4(float w);
// Closed quarter radius 10, straight line 50
Target curva_ala_2_6(float w);

//Circuito 3
// Closed quarter radius 1.0, straight line 50
Target curva_ala_3_1(float w);
// Closed quarter radius 2.5, straight line 50
Target curva_ala_3_2(float w);
// Closed quarter radius 3.0, straight line 50
Target curva_ala_3_3(float w);

//Circuito 4
// Closed semicircle radius 2, straight line 30
Target curva_ala_4_2(float w);
// Closed semicircle radius 3, straight line 30
Target curva_ala_4_3(float w);
// Closed semicircle radius 4, straight line 30
Target curva_ala_4_4(float w);

//Circuito 5
// Closed semicircle radius 4, straight line 30
Target curva_ala_5_4(float w);
// Closed semicircle radius 5, straight line 30
Target curva_ala_5_5(float w);
// Closed semicircle radius 6, straight line 30
Target curva_ala_5_6(float w);

//Circuito 6
// Closed quarter radius 2, straight line 50
Target curva_ala_6_2(float w);
// Closed quarter radius 3, straight line 50
Target curva_ala_6_3(float w);
// Closed quarter radius 4, straight line 50
Target curva_ala_6_4(float w);

//Circuito 7
// Closed quarter radius 4, straight line 50
Target curva_ala_7_4(float w);
// Closed quarter radius 5, straight line 50
Target curva_ala_7_5(float w);
// Closed quarter radius 6, straight line 50
Target curva_ala_7_6(float w);

//Trayectoria Paper
Target curva_lissajous_1(float w);
Target curva_lissajous_2(float w);

#endif