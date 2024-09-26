// Zonotopo.h
#ifndef ZONOTOPO_H
#define ZONOTOPO_H

#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class Zonotopo {
public:
    // Atributos de la clase
    VectorXd c;    // Vector con el centro del zonotopo
    MatrixXd H;    // Matriz de vectores generadores
    int orden;     // Orden del zonotopo (número de vectores generadores)

     // Constructor por defecto
    Zonotopo();

    // Constructor
    Zonotopo(const VectorXd& centro, const MatrixXd& generadores);

    // Método para la reducción de orden
    void reduccionOrden(int s, const MatrixXd& W);

    // Método de filtrado
    static Zonotopo filteringPsi(const Zonotopo& Z, const Vector<double, 1>& y, const Matrix<double, 1,3>& C, const Matrix<double, 1,1>& R, const Matrix <double, 3,3>& W);
    

private:
    // Método para obtener el índice de la columna con la mayor norma
    int mayor(const VectorXd& normas, int r, int j);

    // Método para intercambiar columnas de una matriz y sus respectivas normas
    void swapColumns(MatrixXd& H1, VectorXd& norma, int j, int colMayor);

    static int contarCeros(const MatrixXd& H_fil, int col);
};

#endif // ZONOTOPO_H