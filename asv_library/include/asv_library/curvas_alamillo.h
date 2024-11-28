#ifndef CURVAS_ALAMILLO_H
#define CURVAS_ALAMILLO_H


struct Target {   // Declare target struct type
    float xp;  // Declare member types
    float yp;
    float dxp;
    float dyp;
};

void recta(float w, Target* px, float ax, float by, float mx, float my);
void arco(float w, Target* px, float ax, float by, float r, bool sig) ;


// Closed semicircle radius 6, straight line 20
Target curva0(float w);
// Closed semicircle radius 5, straight line 20
Target curva1(float w);
// Closed semicircle radius 4, straight line 20
Target curva2(float w);
// Closed semicircle radius 3, straight line 20
Target curva3(float w);
// Closed semicircle radius 2, straight line 20
Target curva4(float w);
// Closed circuit of quarter circles radius 6, straight lines 20
Target curva5(float w);
// Closed circuit of quarter circles radius 5, straight lines 20
Target curva6(float w);
// Closed circuit of quarter circles radius 4, straight lines 20
Target curva7(float w);
// Concentric closed circuit of semicircles radius 2, straight lines 20
Target curva8(float w);
// Closed circuit of quarter circles radius 2, straight lines 20
Target curva9(float w);
// Circuit of radius 2 concentric to 4 
Target curva10(const float w);
// Circuit of 4 meters radius
Target curva11(const float w);
// straight line to the south
Target curva12(const float w);

#endif