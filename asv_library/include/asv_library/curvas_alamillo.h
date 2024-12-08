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
// Closed quarter radius 6, straight line 50
Target curva_ala_3_1(float w);
// Closed quarter radius 8, straight line 50
Target curva_ala_3_2(float w);
// Closed quarter radius 10, straight line 50
Target curva_ala_3_3(float w);

#endif