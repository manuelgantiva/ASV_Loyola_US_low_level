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
    while(w >= 12.94984){
        w = w - 12.94984;
    }
    if (w < 3.33333) {
        // Trajectory 1: 20 meters straight line
        recta(w, &result, -6.0, -2.0, -6.0, 0.0);
    } else if (w >= 3.33333 && w < 6.47492) {
        // Trajectory 2: 180 degree arc of circumference radius 6
        arco(w - 3.33333 + 3.14159, &result, −26.0, 4.0, 6.0, true);
    } else if (w >= 6.47492 && w < 9.80825) {
        // Trajectory 3: 20 meters straight line
        recta(w - 6.47492, &result, -26.0, 10.0, 6.0, 0.0);
    } else if (w >= 9.80825 && w < 12.94984) {
        // Trajectory 4: 180 degree arc of circumference radius 6
        arco(w - 9.80825, &result, −6.0, 4.0, 6.0, true);
    }   
    return result;
}

// 
Target curva1(float w) {
    Target result;
    while(w >= 14.28318){
        w = w - 14.28318;
    }
    if (w < 4.0) {
        // Trajectory 1: 20 meters straight line
        recta(w, &result, -6.0, -2.0, -5.0, 0.0);
    } else if (w >= 4.0 && w < 7.14158) {
        // Trajectory 2: 180 degree arc of circumference radius 5
        arco(w - 4.0 + 3.14159, &result, −26.0, 3.0, 5.0, true);
    } else if (w >= 7.14158 && w < 11.14159) {
        // Trajectory 3: 20 meters straight line
        recta(w - 7.14158, &result, -26.0, 8.0, 5.0, 0.0);
    } else if (w >= 11.14159 && w < 14.28318) {
        // Trajectory 4: 180 degree arc of circumference radius 5
        arco(w - 11.14159, &result, −6.0, 3.0, 5.0, true);
    } 
    
    return result;
}

// 
Target curva2(float w) {
    Target result;
    while(w >= 16.28318){
        w = w - 16.28318;
    }
    if (w < 5.0) {
        // Trajectory 1: 20 meters straight line
        recta(w, &result, -6.0, -2.0, -4.0, 0.0);
    } else if (w >= 5.0 && w < 8.14159) {
        // Trajectory 2: 180 degree arc of circumference radius 4
        arco(w - 5.0 + 3.14159, &result, -26.0, 2.0, 4.0, true);
    } else if (w >= 8.14159 && w < 13.14159) {
        // Trajectory 3: 20 meters straight line
        recta(w - 8.14159, &result, -26.0, 6.0, 4.0, 0.0);
    } else if (w >= 13.14159 && w < 16.28318) {
        // Trajectory 4: 180 degree arc of circumference radius 4
        arco(w - 13.14159, &result, −6.0, 2.0, 4.0, true);
    }
    
    return result;
}

// 
Target curva3(float w) {
    Target result;
    while(w >= 19.61650){
        w = w - 19.61650;
    }
    if (w < 6.66666) {
        // Trajectory 1: 20 meters straight line
        recta(w, &result, -6.0, -2.0, -3.0, 0.0);
    } else if (w >= 6.66666 && w < 9.90925) {
        // Trajectory 2: 180 degree arc of circumference radius 3
        arco(w - 6.66666 + 3.14159, &result, -26.0, 1.0, 3.0, true);
    } else if (w >= 9.90925 && w < 16.47491) {
        // Trajectory 3: 20 meters straight line
        recta(w - 9.90925, &result, -26.0, 4.0, 3.0, 0.0);
    } else if (w >= 16.47491 && w < 19.61650) {
        // Trajectory 4: 180 degree arc of circumference radius 3
        arco(w - 16.47491, &result, −6.0, 1.0, 3.0, true);
    }
    
    return result;
}

// 
Target curva4(float w) {
    Target result;
    while(w >= 36.28318){
        w = w - 36.28318;
    }
    if (w < 10.0) {
        // Trajectory 1: 20 meters straight line
        recta(w, &result, -6.0, -2.0, -2.0, 0.0);
    } else if (w >= 10.0 && w < 13.14159) {
        // Trajectory 2: 180 degree arc of circumference radius 2
        arco(w - 10.0 + 3.14159, &result, -26.0, 0.0, 2.0, true);
    } else if (w >= 13.14159 && w < 23.14159) {
        // Trajectory 3: 20 meters straight line
        recta(w - 13.14159, &result, -26.0, 2.0, 2.0, 0.0);
    } else if (w >= 23.14159 && w < 36.28318) {
        // Trajectory 4: 180 degree arc of circumference radius 2
        arco(w - 23.14159, &result, −6.0, 0.0, 2.0, true);
    }
    
    return result;
}

// 
Target curva5(float w) {
    Target result;
    while(w >= 19.6165){
        w = w - 19.6165;
    }
    if (w < 3.33333) {
        // Trajectory 1: 20 meters straight line
        recta(w, &result, 0.0, 6.0, 0.0, -6.0);
    } else if (w >= 3.33333 && w < 4.90413) {
        // Trajectory 2: 180 degree arc of circumference radius 6
        arco(w - 3.33333 + 1.57079, &result, -6.0, -14.0, 6.0, true);
    } else if (w >= 4.90413 && w < 8.23746) {
        // Trajectory 3: 20 meters straight line
        recta(w - 4.90413, &result, -6.0, -20.0, -6.0, 0.0);
    } else if (w >= 8.23746 && w < 9.80826) {
        // Trajectory 4: 180 degree arc of circumference radius 6
        arco(w - 8.23746 + 3.14159, &result, −26.0, -14.0, 6.0, true);
    } else if (w >= 9.80826 && w < 13.14159) {
        // Trajectory 5: 20 meters straight line
        recta(w - 9.80826, &result, -32.0, -14.0, 0.0, 6.0);
    } else if (w >= 13.14159 && w < 14.71238) {
        // Trajectory 6: 180 degree arc of circumference radius 6
        arco(w - 13.14159 - 1.57079, &result, -26.0, 6.0, 6.0, true);
    } else if (w >= 14.71238 && w < 18.04571) {
        // Trajectory 7: 20 meters straight line
        recta(w - 14.71238, &result, -26.0, 12.0, 6.0, 0.0);
    } else if (w >= 18.04571 && w < 19.6165) {
        // Trajectory 8: 180 degree arc of circumference radius 6
        arco(w - 18.04571, &result, −6.0, 6.0, 6.0, true);
    }
    
    return result;
}

// 
Target curva6(float w) {
    Target result;
    while(w >= 22.28318){
        w = w - 22.28318;
    }
    if (w < 4.0) {
        // Trajectory 1: 20 meters straight line
        recta(w, &result, 0.0, 6.0, 0.0, -5.0);
    } else if (w >= 4.0 && w < 5.5708) {
        // Trajectory 2: 180 degree arc of circumference radius 5
        arco(w - 4.0 + 1.57079, &result, -5.0, -14.0, 5.0, true);
    } else if (w >= 5.5708 && w < 9.5708) {
        // Trajectory 3: 20 meters straight line
        recta(w - 5.5708, &result, -5.0, -19.0, -5.0, 0.0);
    } else if (w >= 9.5708 && w < 11.1416) {
        // Trajectory 4: 180 degree arc of circumference radius 5
        arco(w - 9.5708 + 3.14159, &result, −25.0, -14.0, 5.0, true);
    } else if (w >= 11.1416 && w < 15.1416) {
        // Trajectory 5: 20 meters straight line
        recta(w - 11.1416, &result, -30.0, -14.0, 0.0, 5.0);
    } else if (w >= 15.1416 && w < 16.71239) {
        // Trajectory 6: 180 degree arc of circumference radius 5
        arco(w - 15.1416 - 1.57079, &result, -25.0, 6.0, 5.0, true);
    } else if (w >= 16.71239 && w < 20.71239) {
        // Trajectory 7: 20 meters straight line
        recta(w - 16.71239, &result, -25.0, 11.0, 5.0, 0.0);
    } else if (w >= 20.71239 && w < 22.28318) {
        // Trajectory 8: 180 degree arc of circumference radius 5
        arco(w - 20.71239, &result, −5.0, 6.0, 5.0, true);
    }
    
    return result;
}

// 
Target curva7(float w) {
    Target result;
    while(w >= 26.28318){
        w = w - 26.28318;
    }
    if (w < 5.0) {
        // Trajectory 1: 20 meters straight line
        recta(w, &result, 0.0, -6.0, 0.0, -4.0);
    } else if (w >= 5.0 && w < 6.5708) {
        // Trajectory 2: 180 degree arc of circumference radius 5
        arco(w - 5.0 + 1.57079, &result, -4.0, -14.0, 4.0, true);
    } else if (w >= 6.5708 && w < 11.5708) {
        // Trajectory 3: 20 meters straight line
        recta(w - 6.5708, &result, -4.0, -18.0, -4.0, 0.0);
    } else if (w >= 11.5708 && w < 13.1416) {
        // Trajectory 4: 180 degree arc of circumference radius 5
        arco(w - 11.5708 + 3.14159, &result, −24.0, -14.0, 4.0, true);
    } else if (w >= 13.1416 && w < 18.1416) {
        // Trajectory 5: 20 meters straight line
        recta(w - 13.1416, &result, -28.0, -14.0, 0.0, 4.0);
    } else if (w >= 18.1416 && w < 19.71239) {
        // Trajectory 6: 180 degree arc of circumference radius 5
        arco(w - 18.1416 - 1.57079, &result, -24.0, 6.0, 4.0, true);
    } else if (w >= 19.71239 && w < 24.71239) {
        // Trajectory 7: 20 meters straight line
        recta(w - 19.71239, &result, -24.0, 10.0, 4.0, 0.0);
    } else if (w >= 24.71239 && w < 26.28318) {
        // Trajectory 8: 180 degree arc of circumference radius 5
        arco(w - 24.71239, &result, −4.0, 6.0, 4.0, true);
    }
    
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

