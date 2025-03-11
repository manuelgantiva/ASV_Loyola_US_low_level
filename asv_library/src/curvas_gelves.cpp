#include "asv_library/curvas_gelves.h"
#include <cmath> 

void recta(float w, Target* px, float ax, float by, float mx, float my){
    px->yp = by + my * w;
    px->xp = ax + mx * w;
    px->dyp = my;
    px->dxp = mx;
    px->f_c = 1/ std::sqrt(my * my + mx * mx);
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
    px->f_c = 1/r;
}


// Closed quarter radius 6, straight line 30
Target curva_gel_1_6(float w){
    Target result;
    if (w < 7.222222222222221) {
        // Línea recta 
        recta(w, &result, -75.5, 6.8 , -4.5,  0);
    } 
    else if (w >=7.222222222222221 && w < 8.793018549) {
        //curva
        arco(w - 7.222222222222221 + 3.141592653589793, &result,  -108,3.800000000000001, 3.0, false);
    }
    else if (w >= 8.793018549 && w < 13.63746299) {
        // Línea recta 
        recta(w - 8.793018549, &result, -111 , 3.8, 0, -4.5);
    }
    else if (w >=13.63746299 && w < 15.20825932) {
        //curva
        arco(w - 13.63746299 + 1.5707963267948966 , &result,  -117, -17.99999974624237 , 6.0, true);
    }
    else if (w >= 15.20825932 && w < 16.31937043 ) {
        // Línea recta 
        recta(w - 15.20825932 , &result, -117 , -24, -4.4999999999999964, 0);
    }
    else if (w >=16.31937043  && w < 17.89016676) {
        //curva
        arco(w - 16.31937043 - 3.1415926141650266 , &result,  -122.0, -18.000000000000004 , 6.0, true);
    }
    else if (w >= 17.89016676 && w < 26.44572231) {
        // Línea recta 
        recta(w - 17.89016676, &result, -128.0, -18.0 , 0, 4.5);
    }
    else if (w >= 26.44572231 && w < 28.01651864) {
        //curva
        arco(w - 26.44572231 -1.5707963267948966  , &result, -122, 20.4999996945262, 6.0, true);
    }
    else if (w >=28.01651864 && w < 29.59985197) {
        //Línea recta
        arco(w - 28.01651864 , &result,  -122, -26.5 , 6.0,0 );
    }
    else if (w >= 29.59985197&& w < 31.17064834) {
        //curva
        arco(w - 29.59985197  , &result, -112.5, 20.5, 6.0, true);
    }
    else if (w >=31.17064834 && w < 32.88175947) {
        //Línea recta
        arco(w - 31.17064834 , &result,  -106.5, -20.5 , 0,-4.5 );
    }
    else if (w >=32.88175947 && w < 34.4525558) {
        //curva
        arco(w - 32.88175947 - 1.5707963267948966   , &result, -103.5, 12.8, 3.0, false);
    }
    else if (w >=34.4525558) {
        //Línea recta
        arco(w - 34.4525558 , &result,  -103.5, 9.8 ,4.5,0 );
    }
    
    
    return result;
}
// Closed quarter radius 4, straight line 30
Target curva_gel_1_4(float w){
    Target result;
    if (w < 7.222222222222221) {
        // Línea recta 
        recta(w, &result, -75.5, 8.3 , -4.5,  0);
    } 
    else if (w >=7.222222222222221 && w < 8.793018549) {
        //curva
        arco(w - 7.222222222222221 + 3.141592653589793, &result,  -108,3.800000000000001, 4.5, false);
    }
    else if (w >= 8.793018549 && w < 13.63746299) {
        // Línea recta 
        recta(w - 8.793018549, &result, -112.5 , 3.8, 0, -4.5);
    }
    else if (w >=13.63746299 && w < 15.20825932) {
        //curva
        arco(w - 13.63746299 + 1.5707963267948966 , &result,  -117, -17.99999974624237 , 4.5, true);
    }
    else if (w >= 15.20825932 && w < 16.31937043 ) {
        // Línea recta 
        recta(w - 15.20825932 , &result, -117 , -22.5, -4.4999999999999964, 0);
    }
    else if (w >=16.31937043  && w < 17.89016676) {
        //curva
        arco(w - 16.31937043 - 3.1415926141650266 , &result,  -122.0, -18.000000000000004 , 4.5, true);
    }
    else if (w >= 17.89016676 && w < 26.44572231) {
        // Línea recta 
        recta(w - 17.89016676, &result, -126.5, -18.0 , 0, 4.5);
    }
    else if (w >= 26.44572231 && w < 28.01651864) {
        //curva
        arco(w - 26.44572231 -1.5707963267948966  , &result, -122, 20.4999996945262, 4.5, true);
    }
    else if (w >=28.01651864 && w < 29.59985197) {
        //Línea recta
        arco(w - 28.01651864 , &result,  -122, -25 , 6.0,0 );
    }
    else if (w >= 29.59985197&& w < 31.17064834) {
        //curva
        arco(w - 29.59985197  , &result, -112.5, 20.5, 4.5, true);
    }
    else if (w >=31.17064834 && w < 32.88175947) {
        //Línea recta
        arco(w - 31.17064834 , &result,  -108, -20.5 , 0,-4.5 );
    }
    else if (w >=32.88175947 && w < 34.4525558) {
        //curva
        arco(w - 32.88175947 - 1.5707963267948966   , &result, -103.5, 12.8, 4.5, false);
    }
    else if (w >=34.4525558) {
        //Línea recta
        arco(w - 34.4525558 , &result,  -103.5, 8.3 ,4.5,0 );
    }

    return result;
}

// Closed quarter radius 2, straight line 30
Target curva_gel_1_3(float w){
    Target result;
    if (w < 7.222222222222221) {
        // Línea recta 
        recta(w, &result, -75.5, 9.8 , -4.5,  0);
    } 
    else if (w >=7.222222222222221 && w < 8.793018549) {
        //curva
        arco(w - 7.222222222222221 + 3.141592653589793, &result,  -108,3.800000000000001, 6.0, false);
    }
    else if (w >= 8.793018549 && w < 13.63746299) {
        // Línea recta 
        recta(w - 8.793018549, &result, -114 , 3.8, 0, -4.5);
    }
    else if (w >=13.63746299 && w < 15.20825932) {
        //curva
        arco(w - 13.63746299 + 1.5707963267948966 , &result,  -117, -17.99999974624237 , 3.0, true);
    }
    else if (w >= 15.20825932 && w < 16.31937043 ) {
        // Línea recta 
        recta(w - 15.20825932 , &result, -117 , -21, -4.4999999999999964, 0);
    }
    else if (w >=16.31937043  && w < 17.89016676) {
        //curva
        arco(w - 16.31937043 - 3.1415926141650266 , &result,  -122.0, -18.000000000000004 , 3.0, true);
    }
    else if (w >= 17.89016676 && w < 26.44572231) {
        // Línea recta 
        recta(w - 17.89016676, &result, -125.0, -18.0 , 0, 4.5);
    }
    else if (w >= 26.44572231 && w < 28.01651864) {
        //curva
        arco(w - 26.44572231 -1.5707963267948966  , &result, -122, 20.4999996945262, 3.0, true);
    }
    else if (w >=28.01651864 && w < 29.59985197) {
        //Línea recta
        arco(w - 28.01651864 , &result,  -122, -23.5 , 6.0,0 );
    }
    else if (w >= 29.59985197&& w < 31.17064834) {
        //curva
        arco(w - 29.59985197  , &result, -112.5, 20.5, 3.0, true);
    }
    else if (w >=31.17064834 && w < 32.88175947) {
        //Línea recta
        arco(w - 31.17064834 , &result,  -109.5, -20.5 , 0,-4.5 );
    }
    else if (w >=32.88175947 && w < 34.4525558) {
        //curva
        arco(w - 32.88175947 - 1.5707963267948966   , &result, -103.5, 12.8, 6.0, false);
    }
    else if (w >=34.4525558) {
        //Línea recta
        arco(w - 34.4525558 , &result,  -103.5, 6.8 ,4.5,0 );
    }
    
    return result;
}


// Closed semicircle radius 6, straight line 50
Target curva_gel_2_6(float w){
    Target result;
    while(w >= 25.61651864){
        w = w - 25.61651864;
    }
    if (w <8.555555555555555) {
        // Línea recta 
        recta(w, &result, - 111, 20.5, 0, - 4.5);
    } 
    else if (w >= 8.555555555555555 && w <10.12635188 ) {
        //curva
        arco(w -8.555555555555555 + 1.5707963267948966, &result,  -117.0, -17.99999974624237, 6, true);
    }
    else if (w >= 10.12635188 && w < 11.23746299) {
        // Línea recta 
        recta(w - 10.12635188, &result, -117.0 , -24, -4.4999999999999964, 0);
    }
    else if (w >= 11.23746299 && w < 12.80825932) {
        //curva
        arco(w -11.23746299 - 3.1415926141650266, &result,  -122.0, -18.000000000000004, 6, true);
    }
    else if (w >=12.80825932  && w <21.36381488) {
        // Línea recta 
        recta(w -12.80825932, &result, -128 , -18.0, 0, 4.5);
    }
    else if (w >=21.36381488 && w <22.9346112 ) {
        //curva
        arco(w - 21.36381488 -1.5707963267948966 , &result,  -122, 20.4999996945262 , 6, true);
    }
    else if (w >= 22.9346112 && w < 24.04572231) {
        // Línea recta 
        recta(w - 22.9346112, &result, -122 , 26.5, 4.5, 0);
    }
    else if (w >= 24.04572231 && w < 25.61651864) {
        //curva
        arco(w - 24.04572231 , &result,  -117, 20.5, 6, true);
    }
    
    return result;
}
// Closed semicircle radius 8, straight line 50
Target curva_gel_2_4(float w){
    Target result;
    while(w >= 25.61651864){
        w = w - 25.61651864;
    }
    if (w <8.555555555555555) {
        // Línea recta 
        recta(w, &result, - 112.5, 20.5, 0, - 4.5);
    } 
    else if (w >= 8.555555555555555 && w <10.12635188 ) {
        //curva
        arco(w -8.555555555555555 + 1.5707963267948966, &result,  -117.0, -17.99999974624237, 4.5, true);
    }
    else if (w >= 10.12635188 && w < 11.23746299) {
        // Línea recta 
        recta(w - 10.12635188, &result, -117.0 , -22.5, -4.4999999999999964, 0);
    }
    else if (w >= 11.23746299 && w < 12.80825932) {
        //curva
        arco(w -11.23746299 - 3.1415926141650266, &result,  -122.0, -18.000000000000004, 4.5, true);
    }
    else if (w >=12.80825932  && w <21.36381488) {
        // Línea recta 
        recta(w -12.80825932, &result, -126.5 , -18.0, 0, 4.5);
    }
    else if (w >=21.36381488 && w <22.9346112 ) {
        //curva
        arco(w - 21.36381488 -1.5707963267948966 , &result,  -122, 20.4999996945262 , 4.5, true);
    }
    else if (w >= 22.9346112 && w < 24.04572231) {
        // Línea recta 
        recta(w - 22.9346112, &result, -122 , 25, 4.5, 0);
    }
    else if (w >= 24.04572231 && w < 25.61651864) {
        //curva
        arco(w - 24.04572231 , &result,  -117, 20.5, 4.5, true);
    }
    
    return result;
}
// Closed semicircle radius 10, straight line 50
Target curva_gel_2_3(float w){
    Target result;
    while(w >= 25.61651864){
        w = w - 25.61651864;
    }
    if (w <8.555555555555555) {
        // Línea recta 
        recta(w, &result, - 114, 20.5, 0, - 4.5);
    } 
    else if (w >= 8.555555555555555 && w <10.12635188 ) {
        //curva
        arco(w -8.555555555555555 + 1.5707963267948966, &result,  -117.0, -17.99999974624237, 3, true);
    }
    else if (w >= 10.12635188 && w < 11.23746299) {
        // Línea recta 
        recta(w - 10.12635188, &result, -117.0 , -21, -4.4999999999999964, 0);
    }
    else if (w >= 11.23746299 && w < 12.80825932) {
        //curva
        arco(w -11.23746299 - 3.1415926141650266, &result,  -122.0, -18.000000000000004, 3, true);
    }
    else if (w >=12.80825932  && w <21.36381488) {
        // Línea recta 
        recta(w -12.80825932, &result, -125 , -18.0, 0, 4.5);
    }
    else if (w >=21.36381488 && w <22.9346112 ) {
        //curva
        arco(w - 21.36381488 -1.5707963267948966 , &result,  -122, 20.4999996945262 , 3, true);
    }
    else if (w >= 22.9346112 && w < 24.04572231) {
        // Línea recta 
        recta(w - 22.9346112, &result, -122 , 23.5, 4.5, 0);
    }
    else if (w >= 24.04572231 && w < 25.61651864) {
        //curva
        arco(w - 24.04572231 , &result,  -117, 20.5, 3, true);
    }
    
    return result;
}