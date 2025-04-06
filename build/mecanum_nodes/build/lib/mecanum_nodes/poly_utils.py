import numpy as np
from numpy.typing import NDArray

class Polynomial():
    def __init__(self, coefficients: list = None):
        self.coefficients = coefficients

    def evaluate(self, t):
        assert self.coefficients is not None
        x = 0
        for i, c in enumerate(self.coefficients):
            x += c * t**i
        return x
    
    def diff(self, order: int) -> list:
        assert self.coefficients is not None

        coeffs = self.coefficients[:]

        for _ in range(order):
            new_coeffs = []

            for i in range(1, len(coeffs)):
                new_coeffs.append(i * coeffs[i])

            coeffs = new_coeffs

            if not coeffs:
                break  # Stop early if derivative becomes 0

        return coeffs

class PolyTraj():

    def __init__(self, x0: NDArray, xf: NDArray, T=1.0):

        assert np.shape(x0) == np.shape(xf)

        self.T = T

        if len(x0.flatten()) == 2:
            poly_solve = self.solve_cubic

        elif len(x0.flatten()) == 3:
            poly_solve = self.solve_quartic
       
        elif len(x0.flatten()) == 4:
            poly_solve = self.solve_quintic

        self.x: Polynomial = poly_solve(x0, xf)


    def solve_cubic(self, _x0: NDArray, _xf: NDArray) -> NDArray[np.float64]:

        x0, dx0 = _x0.flatten().tolist()
        xf, dxf = _xf.flatten().tolist()

        A = np.array([
            [1,    0,       0,         0      ],   
            [0,    1,       0,         0      ],   
            [1,    self.T,  self.T**2, self.T**3],   
            [0,    1,       2*self.T,  3*self.T**2]    
            ])
        b = np.array([x0, dx0, xf, dxf])

        coeffs = np.linalg.solve(A, b)
        
        return Polynomial(coeffs.tolist())

    def solve_quartic(self, x0, xf) -> Polynomial:
        raise NotImplementedError

    def solve_quintic(self, x0, xf) -> Polynomial:
        raise NotImplementedError
    
    def evaluate(self, t: float, order: int = 0):

        assert order >= 0
        
        if order > 0:
            
            poly = Polynomial(self.x.diff(order))
            return poly.evaluate(t)
           
        else:
            return self.x.evaluate(t)
    
    def get_max_acceleration(self, vel_poly: Polynomial):
        # Get acceleration polynomial: derivative of v(t)
        deg = len(vel_poly.coefficients)
        acc_coeffs = np.array(vel_poly.diff(1))
        acc = Polynomial(acc_coeffs)
        jrk_coeffs = np.array(vel_poly.diff(2))


        # Find roots of a'(t)
        critical_points = np.roots(jrk_coeffs)

        # Keep only real roots in [0, T]
        real_roots = [r.real for r in critical_points if np.isreal(r) and 0 <= r.real <= self.T]

        # Evaluate acceleration at endpoints and critical points

        test_points = [0, self.T] + real_roots
        acc_vals = [acc.evaluate(t) for t in test_points]
        max_acc = max(abs(a) for a in acc_vals)

        return max_acc
