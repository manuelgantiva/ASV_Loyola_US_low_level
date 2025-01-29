#include "asv_library/curvas_alamillo.h"
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

//Trayectoria interna
Target curva_ala_1_2(float w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, 1, -2.0,  0);
    }else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w - 12.5 + 3.12159265, &result,  -31.0, 3.0, 2.0, true);
    }else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 5.0 , 2.0, 0);
    }else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w - 28.14159265 , &result,  -6.0, 3.0, 2.0, true);
    }
    
    return result;
}

//Trayectoria central
Target curva_ala_1_4(float w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, -1.0, -2, 0);
    } else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w - 12.5 + 3.12159265, &result,  -31.0, 3.0, 4.0, true);
    }else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 7, 2, 0);
    }else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w - 28.14159265 , &result,  -6.0, 3.0, 4.0, true);
    }
    return result;
}

//Trayectoria externa
Target curva_ala_1_6(float w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0,-3.0, - 2.0, 0);
    }else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w - 12.5 + 3.12159265, &result,  -31.0, 3.0, 6.0, true);
    }else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 9.0 ,  2.0, 0);
    }else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w - 28.14159265 , &result,  -6.0, 3.0, 6.0, true);
    }
    return result;
}

//Trayectoria interna
Target curva_ala_2_2(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, -4.0, 6.0, 0, - 2.0);
    }else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result,  -6.0,-14.0, 2.0, true);
    }else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -6.0 , -16.0, - 2.0, 0);
    }else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result,  -26.0, -14.0 , 2.0, true);
    }else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265 , &result, -28 , -14, 0, 2.0);
    }else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.5707963267948966, &result,  -26.0, 6.0 , 2.0, true);
    }else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -26.0, 8.0 , 2.0, 0.0);
    }else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result, -6.0, 6.0, 2.0, true);
    }
    return result;
}

//Trayectoria central
Target curva_ala_2_4(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, - 2.0, 6.0, 0, - 2.0);
    }else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result,  -6.0,-14.0, 4.0, true);
    }else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -6.0 , -18.0, -2.0, 0);
    }else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result,  -26.0, -14.0 , 4.0, true);
    }else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265, &result, -30.0 , -14.0, 0, 2.0);
    }else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.5707963267948966, &result,  -26.0, 6.0 , 4.0, true);
    }else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -26 , 10, 2, 0);
    }else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result, -6.0, 6.0, 4.0, true);
    }
    return result;
}

//Trayectoria externa
Target curva_ala_2_6(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, 0,  6.0, 0,  -2);
    }else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result,  -6.0,-14.0, 6.0, true);
    }else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w -11.57079633, &result, -6.0 , -20.0, -2, 0);
    }else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result,  -26.0, -14.0 , 6.0, true);
    }else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265, &result, -32.0 , -14.0 , 0, 2.0);
    }else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.5707963267948966, &result,  -26.0, 6.0 , 6.0, true);
    }else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -26 , 12, 2, 0);
    }else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result, -6.0, 6.0, 6.0, true);
    }    
    return result;
}

//Trayectoria interna
Target curva_ala_3_1(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, -4.0, 6.0, 0,  - 2.0);
    }else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result,  -5.0,-14.0, 1.0, true);
    }else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -5.0 , -15.0, - 2.0, 0);
    }else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result,  -25.0, -14.0 , 1.0, true);
    }else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265 , &result, -26 , -14, 0, 2.0);
    }else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.57079632, &result,  -25.0, 6.0 , 1.0, true);
    }else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -25.0, 7.0 , 2.0, 0);
    }else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result, -5.0, 6.0, 1.0, true);
    }
    
    return result;
}

//Trayectoria central
Target curva_ala_3_2(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, - 2.75, 6.0, 0, - 2.0);
    }else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.57079633, &result,  -5.0,-14.0, 2.25, true);
    }else if (w >= 11.57079633 && w <21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -5.0 , -16.25, -2.0, 0);
    }else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result,  -25.0, -14.0 , 2.25, true);
    }else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265, &result, -27.25 , -14.0, 0, 2.0);
    }else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.57079632, &result,  -25.0, 6.0 , 2.25, true);
    }else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -25 , 8.25, 2, 0);
    }else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result, -5.0, 6.0, 2.25, true);
    }
    return result;
}

//Trayectoria externa
Target curva_ala_3_3(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, - 1.5, 6.0, 0, - 2.0);
    }else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.57079633, &result,  -5.0,-14.0, 3.5, true);
    }else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -5.0 , -17.5, -2.0, 0);
    }else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result,  -25.0, -14.0 , 3.5, true);
    }else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265, &result, -28.5 , -14.0, 0, 2);
    }else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.57079632, &result,  -25.0, 6.0 , 3.5, true);
    }else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -25 , 9.5, 2, 0);
    }else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result,  -5, 6, 3.5, true);
    }
    return result;
}

//Trayectoria interna
Target curva_ala_4_2(float w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, 1.0, -2.0,  0.0);
    } 
    else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w - 12.5 + 3.12159265, &result,  -31.0, 3.0, 2.0, true);
    }
    else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 5.0 , 2.0, 0.0);
    }
    else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w - 28.14159265 , &result,  -6.0 , 3.0, 2.0, true);
    }
    return result;
}

//Trayectoria central
Target curva_ala_4_3(float w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, 0.0, - 2.0 , 0.0);
    } 
    else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w - 12.5 + 3.12159265, &result,  -31.0, 3.0, 3.0, true);
    }
    else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 6.0, 2.0, 0.0);
    }
    else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w -28.14159265, &result,  -6.0, 3.0, 3.0, true);
    }   
    return result;
}

//Trayectoria externa
Target curva_ala_4_4(float w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, -1.0, -2.0, 0.0);
    } 
    else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w -  12.5 + 3.12159265, &result, -31.0, 3.0, 4.0, true);
    }
    else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 7.0, 2.0, 00.0);
    }
    else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w - 28.14159265 , &result, -6.0, 3.0, 4.0, true);
    }
    return result;
}


//Trayectoria interna
Target curva_ala_5_4(float w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, -1.0, -2.0, 0.0);
    } 
    else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w -  12.5 + 3.12159265, &result, -31.0, 3.0, 4.0, true);
    }
    else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0, 7.0, 2.0, 0.0);
    }
    else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w - 28.14159265 , &result, -6.0, 3.0, 4.0, true);
    }
    
    return result;
}

//Trayectoria central
Target curva_ala_5_5(float w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, -2.0, -2.0, 0.0);
    } 
    else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w -12.5 + 3.12159265, &result,  -31.0, 3.0, 5.0, true);
    }
    else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0 , 8.0, 2.0, 0.0);
    }
    else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w -28.14159265, &result, -6.0, 3.0, 5.0, true);
    } 
    return result;
}


//Trayectoria externa
Target curva_ala_5_6(float w) {
    Target result;
    while(w >= 31.28318531){
        w = w - 31.28318531;
    }
    if (w < 12.5) {
        // Línea recta 
        recta(w, &result, -6.0, -3.0, -2.0, 0.0);
    } 
    else if (w >=12.5 && w < 15.64159265) {
        //curva
        arco(w -  12.5 + 3.12159265, &result, -31.0, 3.0, 6.0, true);
    }
    else if (w >= 15.64159265 && w < 28.14159265) {
        // Línea recta 
        recta(w - 15.64159265, &result, -31.0, 9.0, 2.0, 0.0);
    }
    else if (w >=28.14159265 && w < 31.28318531) {
        //curva
        arco(w -28.14159265, &result, -6.0, 3.0, 6.0, true);
    }
    return result;
}


//Trayectoria interna
Target curva_ala_6_2(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, -4.0, 6.0, 0.0, -2.0);
    } 
    else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result,  -6.0,-14.0, 2.0, true);
    }
    else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -6.0 , -16.0, -2.0, 0.0);
    }
    else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result, -26.0, -14.0, 2.0, true);
    }
    else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265 , &result, -28.0, -14.0, 0.0, 2.0);
    }
    else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.5707963267948966, &result, -26.0, 6.0, 2.0, true);
    }
    else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -26.0, 8.0 , 2.0, 0.0);
    }
    else if (w >=44.71238898 && w < 46.28318531){
        //curva
        arco(w - 44.71238898 , &result, -6.0, 6.0, 2.0, true);
    }
    return result;
}


//Trayectoria central
Target curva_ala_6_3(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, -3.0, 6.0, 0.0, -2.0);
    } 
    else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result,  -6.0,-14.0, 3.0, true);
    }
    else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -6.0 , -17.0, -2.0, 0.0);
    }
    else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result,  -26.0, -14.0, 3.0, true);
    }
    else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265 , &result, -29 , -14, 0.0, 2.0);
    }
    else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.5707963267948966, &result,  -26.0, 6.0 , 3.0, true);
    }
    else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -26.0, 9.0 , 2.0, 0.0);
    }
    else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result, -6.0, 6.0, 3.0, true);
    }
    return result;
}


//Trayectoria externa
Target curva_ala_6_4(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, - 2.0, 6.0, 0.0, - 2.0);
    } 
    else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result, -6.0, -14.0, 4.0, true);
    }
    else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -6.0 , -18.0, -2.0, 0.0);
    }
    else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w -21.57079633 + 3.141592653, &result,  -26.0, -14.0, 4.0, true);
    }
    else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265, &result, -30.0 , -14.0, 0.0, 2.0);
    }
    else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w -   33.14159265 - 1.5707963267948966, &result,  -26.0, 6.0, 4.0, true);
    }
    else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -26 , 10, 2.0, 0.0);
    }
    else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result,  -6, 6, 4.0, true);
    }
    return result;
}

//Trayectoria interna
Target curva_ala_7_4(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, - 2.0, 6.0, 0.0, - 2.0);
    } 
    else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result,  -6.0, -14.0, 4.0, true);
    }
    else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -6.0 , -18.0, -2.0, 0);
    }
    else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w -21.57079633 + 3.141592653, &result, -26.0, -14.0, 4.0, true);
    }
    else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265, &result, -30.0 , -14.0, 0, 2.0);
    }
    else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w -   33.14159265 - 1.5707963267948966, &result, -26.0, 6 , 4.0, true);
    }
    else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -26, 10, 2.0, 0.0);
    }
    else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result,  -6, 6, 4.0, true);
    }
    
    return result;
}


//Trayectoria radio 5
Target curva_ala_7_5(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, -1.0, 6.0, 0.0, -2.0);
    } 
    else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result, -6.0, -14.0, 5.0, true);
    }
    else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w - 11.57079633, &result, -6.0, -19.0, -2.0, 0.0);
    }
    else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result, -26.0, -14.0, 5.0, true);
    }
    else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265 , &result, -31 , -14, 0.0, 2.0);
    }
    else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.5707963267948966, &result, -26.0, 6.0, 5.0, true);
    }
    else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -26.0, 11.0 , 2.0, 0.0);
    }
    else if (w >=44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result, -6.0, 6.0, 5.0, true);
    }
    
    return result;
}


//Trayectoria externa
Target curva_ala_7_6(float w) {
    Target result;
    while(w >= 46.28318531){
        w = w - 46.28318531;
    }
    if (w < 10) {
        // Línea recta 
        recta(w, &result, 0,  6.0, 0.0, -2.0);
    } 
    else if (w >=10 && w < 11.57079633) {
        //curva
        arco(w - 10 + 1.5707963267948966, &result,  -6.0, -14, 6.0, true);
    }
    else if (w >= 11.57079633 && w < 21.57079633) {
        // Línea recta 
        recta(w -11.57079633, &result, -6.0 , -20.0, -2.0, 0.0);
    }
    else if (w >=21.57079633 && w < 23.14159265) {
        //curva
        arco(w - 21.57079633 + 3.141592653, &result,  -26.0, -14, 6.0, true);
    }
    else if (w >= 23.14159265 && w < 33.14159265) {
        // Línea recta 
        recta(w - 23.14159265, &result, -32.0 , -14.0 , 0.0, 2.0);
    }
    else if (w >=33.14159265 && w < 34.71238898) {
        //curva
        arco(w - 33.14159265 - 1.5707963267948966, &result,  -26.0, 6.0, 6.0, true);
    }
    else if (w >= 34.71238898 && w < 44.71238898) {
        // Línea recta 
        recta(w - 34.71238898, &result, -26 , 12, 2.0, 0.0);
    }
    else if (w >= 44.71238898 && w < 46.28318531) {
        //curva
        arco(w - 44.71238898 , &result,  -6.0, 6.0, 6.0, true);
    }
    return result;
}

//Trayectoria Paper 1
Target curva_lissajous_1(float w){
    Target result;
    result.yp = -2.0 + 15.0 * sin(-w + 1.5707963268); 
    result.xp = -12.0 + 10.0 * sin(-2.0 * w); 
    result.dyp = -15.0 * sin(w);
    result.dxp = -20.0 * cos(-2.0 * w); 
    result.f_c = 1; // no se utiliza en el MLC
    return result;
}

//Trayectoria Paper 2
Target curva_lissajous_2(float w){
    Target result;
    result.yp = -2.0 + 12.0 * sin(-w + 1.5707963268); 
    result.xp = -12.0 + 10.0 * sin(-2.0 * w); 
    result.dyp = -12.0 * sin(w);
    result.dxp = -20.0 * cos(-2.0 * w); 
    result.f_c = 1; // no se utiliza en el MLC
    return result;
}