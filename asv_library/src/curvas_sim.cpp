#include "asv_library/curvas_sim.h"
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

Target curva_sim_1_6(float w) {
    Target result;
    while(w >= 26.283185307179586){
        w = w - 26.283185307179586;
    }
    if (w < 5.0) {
        // Trayectoria 1: Línea recta de (20, 10) a (20, 40) en MATLAB
        recta(w, &result, 10.0, 20.0, 6.0, 0.0);
    } 
    else if (w >= 5.0 && w < 6.5707963267948966) {
        // Trayectoria 2: Cuarto de circunferencia de (20, 40) a (26, 46) en MATLAB
        arco(w - 5.0, &result, 40.0, 26.0, 6.0, false);
    }
    else if (w >= 6.5707963267948966 && w < 11.5707963267948966) {
        // Trayectoria 3: Línea recta de (26, 46) a (56, 46) en MATLAB
        recta(w - 6.5707963267948966, &result, 46.0, 26.0, 0.0, 6.0);
    }
    else if (w >= 11.5707963267948966 && w < 13.141592653589793) {
        // Trayectoria 4: Cuarto de circunferencia de (56, 46) a (62, 40) en MATLAB
        arco(w - 11.5707963267948966 + 1.5707963267948966, &result, 40.0, 56.0, 6.0, false);
    }
    else if (w >= 13.141592653589793 && w < 18.141592653589793) {
        // Trayectoria 5: Línea recta de (62, 40) a (62, 10) en MATLAB
        recta(w - 13.141592653589793, &result, 40.0, 62.0, -6.0, 0.0);
    }
    else if (w >= 18.141592653589793 && w < 19.71238898038469) {
        // Trayectoria 6: Cuarto de circunferencia de (62, 10) a (56, 4) en MATLAB
        arco(w - 18.141592653589793 + 3.141592653589793, &result, 10.0, 56.0, 6.0, false);
    }
    else if (w >= 19.71238898038469 && w < 24.71238898038469) {
        // Trayectoria 7: Línea recta de (56, 4) a (26, 4) en MATLAB
        recta(w - 19.71238898038469, &result, 4.0, 56.0, 0.0, -6.0);
    }
    else if (w >= 24.71238898038469 && w < 26.283185307179586) {
        // Trayectoria 8: Cuarto de circunferencia de (26, 4) a (20, 10) en MATLAB
        arco(w - 24.71238898038469 - 1.5707963267948966, &result, 10.0, 26.0, 6.0, false);
    }
    return result;
}

Target curva_sim_1_4(float w) {
    Target result;
    while(w >= 36.283185307179586){
        w = w - 36.283185307179586;
    }
    if (w < 7.5) {
        // Trayectoria 1: Línea recta de (22, 10) a (22, 40) en MATLAB
        recta(w, &result, 10.0, 22.0, 4.0, 0.0);
    } 
    else if (w >= 7.5 && w < 9.070796326794897) {
        // Trayectoria 2: Cuarto de circunferencia de (22, 40) a (26, 44) en MATLAB
        arco(w - 7.5, &result, 40.0, 26.0, 4.0, false);
    }
    else if (w >= 9.070796326794897 && w < 16.570796326794897) {
        // Trayectoria 3: Línea recta de (26, 44) a (56, 44) en MATLAB
        recta(w - 9.070796326794897, &result, 44.0, 26.0, 0.0, 4.0);
    }
    else if (w >= 16.570796326794897 && w < 18.141592653589793) {
        // Trayectoria 4: Cuarto de circunferencia de (56, 44) a (60, 40) en MATLAB
        arco(w - 16.570796326794897 + 1.5707963267948966, &result, 40.0, 56.0, 4.0, false);
    }
    else if (w >= 18.141592653589793 && w < 25.641592653589793) {
        // Trayectoria 5: Línea recta de (60, 40) a (60, 10) en MATLAB
        recta(w - 18.141592653589793, &result, 40.0, 60.0, -4.0, 0.0);
    }
    else if (w >= 25.641592653589793 && w < 27.21238898038469) {
        // Trayectoria 6: Cuarto de circunferencia de (60, 10) a (56, 6) en MATLAB
        arco(w - 25.641592653589793 + 3.141592653589793, &result, 10.0, 56.0, 4.0, false);
    }
    else if (w >= 27.21238898038469 && w < 34.71238898038469) {
        // Trayectoria 7: Línea recta de (56, 6) a (26, 6) en MATLAB
        recta(w - 27.21238898038469, &result, 6.0, 56.0, 0.0, -4.0);
    }
    else if (w >= 34.71238898038469 && w < 36.283185307179586) {
        // Trayectoria 8: Cuarto de circunferencia de (26, 6) a (22, 10) en MATLAB
        arco(w - 34.71238898038469 - 1.5707963267948966, &result, 10.0, 26.0, 4.0, false);
    }
    
    return result;
}


Target curva_sim_1_2(float w) {
    Target result;
    while(w >= 66.283185307179586){
        w = w - 66.283185307179586;
    }
    if (w < 15.0) {
        // Trayectoria 1: Línea recta de (24, 10) a (24, 40) en MATLAB
        recta(w, &result, 10.0, 24.0, 2.0, 0.0);
    } 
    else if (w >= 15.0 && w < 16.570796326794897) {
        // Trayectoria 2: Cuarto de circunferencia de (24, 40) a (26, 42) en MATLAB
        arco(w - 15.0, &result, 40.0, 26.0, 2.0, false);
    }
    else if (w >= 16.570796326794897 && w < 31.570796326794897) {
        // Trayectoria 3: Línea recta de (26, 42) a (56, 42) en MATLAB
        recta(w - 16.570796326794897, &result, 42.0, 26.0, 0.0, 2.0);
    }
    else if (w >= 31.570796326794897 && w < 33.141592653589793) {
        // Trayectoria 4: Cuarto de circunferencia de (56, 42) a (58, 40) en MATLAB
        arco(w - 31.570796326794897 + 1.5707963267948966, &result, 40.0, 56.0, 2.0, false);
    }
    else if (w >= 33.141592653589793 && w < 48.141592653589793) {
        // Trayectoria 5: Línea recta de (58, 40) a (58, 10) en MATLAB
        recta(w - 33.141592653589793, &result, 40.0, 58.0, -2.0, 0.0);
    }
    else if (w >= 48.141592653589793 && w < 49.71238898038469) {
        // Trayectoria 6: Cuarto de circunferencia de (58, 10) a (56, 8) en MATLAB
        arco(w - 48.141592653589793 + 3.141592653589793, &result, 10.0, 56.0, 2.0, false);
    }
    else if (w >= 49.71238898038469 && w < 64.71238898038469) {
        // Trayectoria 7: Línea recta de (56, 8) a (26, 8) en MATLAB
        recta(w - 49.71238898038469, &result, 8.0, 56.0, 0.0, -2.0);
    }
    else if (w >= 64.71238898038469 && w < 66.283185307179586) {
        // Trayectoria 8: Cuarto de circunferencia de (26, 8) a (24, 10) en MATLAB
        arco(w - 64.71238898038469 - 1.5707963267948966, &result, 10.0, 26.0, 2.0, false);
    }
    
    return result;
}


//Trayectoria interna
Target curva_sim_2_6(float w) {
    Target result;
    while(w >= 22.94985){
        w = w - 22.94985;
    }
    if (w < 8.33334) {
        // Línea recta 
        recta(w, &result, 50, 20, 0,  6);
    } 
    else if (w >=8.33334 && w < 11.47493) {
        //curva
        arco(w - 8.33334 + 1.5707963267948966, &result,  44, 70.0, 6.0, false);
    }
    else if (w >= 11.47493 && w < 19.80826) {
        // Línea recta 
        recta(w - 11.47493, &result, 38 , 70, 0, -6.0);
    }
    else if (w >=19.80826 && w < 22.94985) {
        //curva
        arco(w - 19.80826 - 1.5707963267948966, &result,  44, 20.0, 6.0, false);
    }
    return result;
}

//Trayectoria central
Target curva_sim_2_8(float w) {
    Target result;
    while(w >= 22.94985){
        w = w - 22.94985;
    }
    if (w < 8.33333) {
        // Línea recta 
        recta(w, &result, 52, 20, 0,  6);
    } 
    else if (w >=8.33333 && w < 11.47493) {
        //curva
        arco(w - 8.33333 + 1.5707963267948966, &result,  44, 70.0, 8.0, false);
    }
    else if (w >= 11.47493 && w < 19.80826) {
        // Línea recta 
        recta(w - 11.47493, &result, 36 , 70, 0, -6.0);
    }
    else if (w >=19.80826 && w < 22.94985) {
        //curva
        arco(w - 19.80826 - 1.5707963267948966, &result,  44, 20.0, 8.0, false);
    }
    return result;
}

//Trayectoria externa
Target curva_sim_2_10(float w) {
    Target result;
    while(w >= 22.94985){
        w = w - 22.94985;
    }
    if (w < 8.33333) {
        // Línea recta 
        recta(w, &result, 54, 20, 0,  6);
    } 
    else if (w >=8.33333 && w < 11.47493) {
        //curva
        arco(w - 8.33333 + 1.5707963267948966, &result,  44, 70.0, 10.0, false);
    }
    else if (w >= 11.47493 && w < 19.80826) {
        // Línea recta 
        recta(w - 11.47493, &result, 34 , 70, 0, -6.0);
    }
    else if (w >=19.80826 && w < 22.94985) {
        //curva
        arco(w - 19.80826 - 1.5707963267948966, &result,  44, 20.0, 10.0, false);
    }
    return result;
}

//Trayectoria interna
Target curva_sim_3_6(float w) {
    Target result;
    while(w >= 39.61651){
        w = w - 39.61651;
    }
    if (w < 8.33333) {
        recta(w, &result, 50, 20, 0,  6);
    } 
    else if (w >=8.33333 && w < 9.90413) {
        arco(w - 8.33333 - 1.5707963267948966, &result,  56, 70.0, 6.0, true);
    }
    else if (w >= 9.90413 && w < 18.23746) {
        recta(w - 9.90413, &result, 56 , 76, 6.0, 0);
    }
    else if (w >= 18.23746 && w < 19.80826) {
        arco(w - 18.23746, &result,  106.0, 70.0, 6.0, true);
    }
    else if (w >= 19.80826 && w < 28.14159) {
        recta(w - 19.80826, &result, 112 , 70, 0, -6.0);
    }
    else if (w >= 28.14159 && w < 29.71238) {
        arco(w - 28.14159 + 1.5707963267948966, &result,  106.0, 20.0 , 6.0, true);
    }
    else if (w >= 29.71238 && w < 38.04571) {
        recta(w - 29.71238, &result, 106 , 14, -6, 0);
    }
    else if (w >= 38.04571 && w < 39.61651) {
        arco(w - 38.04571 + 3.141592653589793, &result,  56.0, 20.0, 6.0, true);
    }
    return result;
}

//Trayectoria central
Target curva_sim_3_8(float w) {
    Target result;
    while (w >= 39.61652) { 
        w -= 39.61652;
    }

    if (w < 8.33333) {
        recta(w, &result, 48, 20, 0, 6.0);
    }
    else if (w >= 8.33333 && w < 9.90413) {  
        arco(w - 8.33333 - 1.57080, &result, 56, 70.0, 8.0, true);
    } 
    else if (w >= 9.90413 && w < 18.23746) {  
        recta(w - 9.90413, &result, 56, 78, 6.0, 0);
    } 
    else if (w >= 18.23746 && w < 19.80826) {  
        arco(w - 18.23746, &result, 106.0, 70.0, 8.0, true);
    } 
    else if (w >= 19.80826 && w < 28.14159) {  
        recta(w - 19.80826, &result, 114, 70, 0, -6.0);
    } 
    else if (w >= 28.14159 && w < 29.71239) {  
        arco(w - 28.14159 + 1.57080, &result, 106.0, 20.0, 8.0, true);
    } 
    else if (w >= 29.71239 && w < 38.04572) {  
        recta(w - 29.71239, &result, 106, 12, -6.0, 0);
    } 
    else if (w >= 38.04572 && w < 39.61652) { 
        arco(w - 38.04572 + 3.14159, &result, 56.0, 20.0, 8.0, true);
    }
    return result;
}

Target curva_sim_3_10(float w) {
    Target result;
    while (w >= 39.61652) {
        w -= 39.61652;
    }

    if (w < 8.33333) {
        recta(w, &result, 46, 20, 0, 6);
    } 
    else if (w >= 8.33333 && w < 9.90413) {
        arco(w - 8.33333 - 1.57080, &result, 56, 70.0, 10.0, true);
    } 
    else if (w >= 9.90413 && w < 18.23746) {
        recta(w - 9.90413, &result, 56, 80, 6.0, 0);
    } 
    else if (w >= 18.23746 && w < 19.80826) {
        arco(w - 18.23746, &result, 106.0, 70.0, 10.0, true);
    } 
    else if (w >= 19.80826 && w < 28.14159) {
        recta(w - 19.80826, &result, 116, 70, 0, -6.0);
    } 
    else if (w >= 28.14159 && w < 29.71239) {
        arco(w - 28.14159 + 1.57080, &result, 106.0, 20.0, 10.0, true);
    } 
    else if (w >= 29.71239 && w < 38.04572) {
        recta(w - 29.71239, &result, 106, 10, -6, 0);
    } 
    else if (w >= 38.04572 && w < 39.61652) {
        arco(w - 38.04572 + 3.14159, &result, 56.0, 20.0, 10.0, true);
    }
    return result;
}