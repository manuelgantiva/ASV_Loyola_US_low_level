#ifndef CURVAS_GELVES_H
#define CURVAS_GELVES_H


struct Target {   // Declare target struct type
    float xp;  // Declare member types
    float yp;
    float dxp;
    float dyp;
    float f_c;
};

void recta(float w, Target* px, float ax, float by, float mx, float my);
void arco(float w, Target* px, float ax, float by, float r, bool sig) ;


// Closed quarter radius 6, straight line 30
Target curva_gel_1_6(float w);
// Closed quarter radius 4, straight line 30
Target curva_gel_1_4(float w);
// Closed quarter radius 2, straight line 30
Target curva_gel_1_3(float w);

// Closed semicircle radius 6, straight line 50
Target curva_gel_2_6(float w);
// Closed semicircle radius 8, straight line 50
Target curva_gel_2_4(float w);
// Closed semicircle radius 10, straight line 50
Target curva_gel_2_3(float w);



#endif