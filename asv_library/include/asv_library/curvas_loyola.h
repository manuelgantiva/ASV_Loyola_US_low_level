#ifndef CURVAS_LOYOLA_H
#define CURVAS_LOYOLA_H


struct Target {   // Declare target struct type
    float xp;  // Declare member types
    float yp;
    float dxp;
    float dyp;
};

void recta(float w, Target* px, float ax, float by, float mx, float my);
void arco(float w, Target* px, float ax, float by, float r, bool sig) ;


// Closed semicircle radius 6, straight line 20
Target curva_1_i(const float w);
// Closed semicircle radius 5, straight line 20
Target curva_1_c(const float w);
// Closed semicircle radius 4, straight line 20
Target curva_1_d(const float w);


#endif