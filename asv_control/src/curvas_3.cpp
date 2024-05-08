#include "curvas.hpp"
#include <cmath> 

void recta(float w, Target* px, float ax, float by, float mx, float my){
    px->yp = by + my * w;
    px->xp = ax + mx * w;
    px->dyp = my;
    px->dxp = mx;
}

void arco(float w, Target* px, float ax, float by, float r, bool sig) {
    if(sig){
        px->yp = by + r * cos(w);
        px->dyp = -r * sin(w);
    }else{
        px->yp = by - r * cos(w);
        px->dyp = r * sin(w);
    }
    px->xp = ax + r * sin(w);
    px->dxp = r * cos(w);
}

// 
Target curva0(float w) {
    Target result;
    if (w < 7.64199) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, -2.0, 2.48626, -0.26171);
    } else if (w >= 7.64199 && w < 10.78358) {
        // Trajectory 2: 180 degree arc of circumference
        arco(w - 7.64199 + 0.10488, &result, 18.73829, -6.48626 , 2.5, true);
    } else if (w >= 10.78358) {
        // Trajectory 3: 8 meter straight line
        recta(w - 10.78358, &result, 18.47658, -8.97253, -2.48626, 0.26171);
    }   
    return result;
}

// 
Target curva1(float w) {
    Target result;
    
    return result;
}

// 
Target curva2(float w) {
    Target result;
    
    return result;
}

// 
Target curva3(float w) {
    Target result;
    
    return result;
}

// 
Target curva4(float w) {
    Target result;
    
    return result;
}

// 
Target curva5(float w) {
    Target result;
    
    return result;
}

// 
Target curva6(float w) {
    Target result;
    
    return result;
}

// 
Target curva7(float w) {
    Target result;
    
    return result;
}

// 
Target curva8(float w) {
    Target result;
    
    return result;
}

// 
Target curva9(float w) {
    Target result;
    
    return result;
}

// 
Target curva10(float w) {
    Target result;
    
    return result;
}

// 
Target curva11(float w) {
    Target result;
    
    return result;
}

