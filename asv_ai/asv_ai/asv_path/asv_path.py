import numpy as np

# todo hecho en ros ya con trayectorias mejores



# Tareyectoria parametrizada por theta. El parametro puede controlar la forma de la trayectoria (lider virtual)
class ParametrizedPath:
    # parametrized by theta
    # Inicializa la instancia de ParametrizedPath
    def __init__(self,path_no=0, theta=0): # path_no: 0, 1, 2 identifica la trayectoria
        self.theta = theta # Actua como índice o parametro para calcular las coordenadas de la trayectoria
        self.path_no = path_no

    def _scale_coordinates(self, theta):
        """Scale theta parameter to reasonable coordinate range"""
        scale_factor = 0.3  # Maps larger theta to reasonable coordinates
        offset = -2.0       # Center around origin
        return theta * scale_factor + offset

    # Esto en ros ya esta hecho con trayectorias mejores
    # Devuelve las coordenadas de la trayectoria en función de theta
    def path(self, theta, deriv=False):
        scaled_theta = self._scale_coordinates(theta)
        match self.path_no:
            case 0: # Trayectoria lineal diagonal
                if deriv: # Si es true devuelve la derivada de la trayectoria, en caso contrario solo las coordenadas
                    return np.array([[scaled_theta], [scaled_theta]]), np.array([0.3])  # scaled derivative
                else:
                    return np.array([[scaled_theta], [scaled_theta]])
            case 1: # Trayectoria lineal con pendiente 0.5
                if deriv:
                    return np.array([[scaled_theta], [0.5*scaled_theta]]), np.array([0.15])  # scaled derivative
                else:
                    return np.array([[scaled_theta], [0.5*scaled_theta]])
            case _: # Trayectoria lineal diagonal igual que el caso 0
                if deriv:
                    return np.array([[scaled_theta], [scaled_theta]]), np.array([0.3])  # scaled derivative
                else:
                    return np.array([[scaled_theta], [scaled_theta]])

    # find the projection x_p of point x_c along the path at theta
    def projection(self, x_c):

        match self.path_no:
            case 0:
                a, b= (x_c[0] + x_c[1])/2, (x_c[0] + x_c[1])/2
            case 1:
                a, b= (0.8*x_c[0] + 0.4*x_c[1]), (0.8*x_c[0] + 0.4*x_c[1])*0.5
            case _:
                a, b= (x_c[0] + x_c[1])/2, (x_c[0] + x_c[1])/2

        return np.array([a, b])

    # find the along track error e = ||x_p - x(theta)||
    def along_track_error(self, x_c):
        return np.linalg.norm(self.projection(x_c) - self.path(self.theta))

    # find the cross track error e = ||x_p - x_c||
    def cross_track_error(self, x_c):
        return np.linalg.norm(self.projection(x_c) - x_c)

    # find the state of the path following system
    def state(self, x_c):
        return self.cross_track_error(x_c)

if __name__ == '__main__':

    # TODO: subir a github
    import matplotlib.pyplot as plt
    import numpy as np

    # Crea la instancia de ParametrizedPath con path_no=0 (trayectoria lineal diagonal) y theta=0
    param_p = ParametrizedPath(0)
    thethas = np.linspace(0, 10, 100)
    x_vs = np.array([param_p.path(thetha) for thetha in thethas])
    plt.plot(x_vs[:, 0], x_vs[:,1], '-k', label='path') # Grafica la trayqectoria en negro (-k) y le asigna la etiqueta 'path'

    param_p.theta = thethas[50] # Asigna un valor intermedio como la posicion actual del lider virtual
    #Virtual leader position
    x_v = param_p.path(param_p.theta)
    plt.plot(x_v[0], x_v[1], 'or', label='x_v')

    #Current position
    x_c = np.array([[1], [4]])
    plt.plot(x_c[0], x_c[1], 'ob', label='x_c')

    #Projection of x_c on the path
    x_p = param_p.projection(x_c)
    plt.plot(x_p[0], x_p[1], 'og', label='x_p')
    plt.plot(np.array([x_p[0], x_c[0]]), np.array([x_p[1], x_c[1]]), '--g', label='x_p')

    # Imprime y grfica el error de cruce (distancia de posicion actual a la proyeccion en la ruta)
    print(param_p.cross_track_error(x_c))
    plt.axis('equal') # Ajusta la escala de los ejes, muestra la leyenda de etieuqteas de puntos y trayectoria y renderiza en pantalla
    plt.legend()
    plt.show()
