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
    static Zonotopo filteringP(const Zonotopo& Z, const Vector<double, 2>& y, const Matrix<double, 2,6>& C, const Matrix<double, 2,2>& R, const Matrix <double, 6,6>& W);

    static Zonotopo prediction_Y(const MatrixXd& Ap, const Zonotopo& Z, 
                          double min_psi, double max_psi, 
                          const MatrixXd& Bwp, const MatrixXd& Qp, 
                          const MatrixXd& Bp_tau);
private:
    // Método para obtener el índice de la columna con la mayor norma
    int mayor(const VectorXd& normas, int r, int j);
    // Método para intercambiar columnas de una matriz y sus respectivas normas
    void swapColumns(MatrixXd& H1, VectorXd& norma, int j, int colMayor);
    static int contarCeros(const MatrixXd& H_fil, int col);
    static std::pair<MatrixXd, MatrixXd> rad_mid(const MatrixXd& Ap, double min_psi, double max_psi);
    static MatrixXd rs(const MatrixXd& H);
    static MatrixXd int_prod(const MatrixXd& a, const MatrixXd& b);
    static std::pair<MatrixXd, MatrixXd> max_min_trig(double min_psi, double max_psi);
    static MatrixXd max_min(int ind, const Zonotopo& Z);
};

#endif // ZONOTOPO_H