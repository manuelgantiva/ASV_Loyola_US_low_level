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

// Straight line
Target curva0(float w) {
    Target result;
    if (w < 10.82959) {
        // Trajectory 1: Straight line
        recta(w, &result, 0.0, -4.0, 2.49317, -0.18468);
    }   
    return result;
}

// Circuit of two straight lines with semicircles radius 2.5
Target curva1(float w) {
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

// Straight line
Target curva2(float w) {
    Target result;
    if (w < 12.93626) {
        // Trajectory 1: Straight line
        recta(w, &result, -2.0, -5.0, 0.69572, -0.07730);
    } else if (w >= 12.93626 && w < 13.24816) {
        // Trajectory 2: 180 degree arc of circumference
        arco(w - 12.93626 - 0.11066, &result, 7.07730, -5.30428 , 0.7, false);
    } else if (w >= 13.24816 && w < 16.3553) {
        // Trajectory 3: 15 meter straight line
        recta(w - 13.24816, &result, 7.21722, -5.99015, 0.68589, 0.13992);
    } else if (w >= 16.3553 && w < 16.26472) {
        // Trajectory 4: 180 degree arc of circumference
        arco(w - 16.3553 - 0.20124, &result, 9.48825, -6.24126 , 0.7, true);
    } else if (w >= 16.26472) {
        // Trajectory 5: 15 meter straight line
        recta(w - 16.26472, &result, 9.56555, -5.54554, 0.69572, -0.07730);
    }
    return result;
}