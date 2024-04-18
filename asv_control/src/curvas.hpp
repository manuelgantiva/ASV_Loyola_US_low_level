#ifndef CURVAS_HPP
#define CURVAS_HPP



struct Target {   // Declare target struct type
    float xp;  // Declare member types
    float yp;
    float dxp;
    float dyp;
};

void recta(float w, Target* px, float ax, float by, float mx, float my);
void arco(float w, Target* px, float ax, float by, float r, bool sig) ;

Target curva1(float w);
Target curva2(float w); 
Target curva3(float w);
Target curva4(float w); 
Target curva5(float w);
Target curva6(float w); 
Target curva7(float w); 
Target curva8(float w);
Target curva9(float w); 

#endif