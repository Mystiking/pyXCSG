import numpy as np

# A grid is always created based on a Signed Distance Field 'phi'
class Grid:
    def __getitem__(grid, grid_point):
        for ii in range(grid.m_I):
            if grid.m_min_coord[0] + grid.m_delta[0] * ii == grid_point[0]:
                break
        for jj in range(grid.m_J):
            if grid.m_min_coord[1] + grid.m_delta[1] * jj == grid_point[1]:
                break
        for kk in range(grid.m_K):
            if grid.m_min_coord[2] + grid.m_delta[2] * kk == grid_point[2]:
                break
        return grid.values[(kk*grid.m_J + jj)*grid.m_I + ii]


    def __init__(self, m_min_coord, m_max_coord, m_I, m_J, m_K, values=None, phi=None):
        self.m_min_coord = m_min_coord # Vector3 coord
        self.m_max_coord = m_max_coord # Vector3 coord
        self.m_I = m_I # Number of nodes along x_axis
        self.m_J = m_J # Number of nodes along y-axis
        self.m_K = m_K # Number of nodes along z-axis
        self.m_delta = [(m_max_coord[0]-m_min_coord[0])/(m_I-1.0),
                        (m_max_coord[1]-m_min_coord[1])/(m_J-1.0),
                        (m_max_coord[2]-m_min_coord[2])/(m_K-1.0)] # Spacing between nodes
        if values is None:
            self.values = np.zeros((self.m_I * self.m_J * self.m_K, 1))
        else:
            self.values = values
        self.phi = phi

    # Fill out self.value?
    def create(self, phi):
        self.phi = phi
        for ii in range(self.m_I):
            for jj in range(self.m_J):
                for kk in range(self.m_K):
                    idx = (kk*self.m_J + jj)*self.m_I + ii
                    self.values[idx] =\
                            phi(np.array([self.m_min_coord[0] + self.m_delta[0] * ii,
                                          self.m_min_coord[1] + self.m_delta[1] * jj,
                                          self.m_min_coord[2] + self.m_delta[2] * kk]))

def sphere(delta):
    return lambda p: np.sqrt((p**2)) - delta

def box(w, h, d):
    def corner_check_return(p):
        if p[0] > 0 and p[1] > 0 and p[2] > 0:
            return np.sqrt(np.dot(p.T, p))
        else:
            return np.max(p)

    return lambda x: corner_check_return(np.array([np.abs(x[0])-w, np.abs(x[1])-h,np.abs(x[2])-d]))

def cylinder(h, r):
    def corner_check_return(p):
        if p[0] > 0 and p[1] > 0:
            return np.sqrt(np.dot(p.T, p))
        else:
            return max(p)
    return lambda x: corner_check_return(np.array([np.sqrt(x[0]**2 + x[1]**2) - r, x[2]-h/2.]))

def union(A, B):
    # Assert that dimensions match
    assert(A.m_I == B.m_I)
    assert(A.m_J == B.m_J)
    assert(A.m_K == B.m_K)
    assert(A.m_min_coord == B.m_min_coord and A.m_max_coord == B.m_max_coord)
    min_coord = A.m_min_coord
    max_coord = A.m_max_coord
    I = A.m_I
    J = A.m_J
    K = A.m_K
    phi = lambda x: min(A.phi(x), B.phi(x)) # Union => min(A(x), B(x))
    C = Grid(min_coord, max_coord, I, J, K, phi=phi)
    return C

def intersection(A, B):
    # Assert that dimensions match
    assert(A.m_I == B.m_I)
    assert(A.m_J == B.m_J)
    assert(A.m_K == B.m_K)
    assert(A.m_min_coord == B.m_min_coord and A.m_max_coord == B.m_max_coord)
    min_coord = A.m_min_coord
    max_coord = A.m_max_coord
    I = A.m_I
    J = A.m_J
    K = A.m_K
    phi = lambda x: max(A.phi(x), B.phi(x)) # Intersection => max(A(x), B(x))
    C = Grid(min_coord, max_coord, I, J, K, phi=phi)
    return C

def difference(A, B):
    # Assert that dimensions match
    assert(A.m_I == B.m_I)
    assert(A.m_J == B.m_J)
    assert(A.m_K == B.m_K)
    assert(A.m_min_coord == B.m_min_coord and A.m_max_coord == B.m_max_coord)
    min_coord = A.m_min_coord
    max_coord = A.m_max_coord
    I = A.m_I
    J = A.m_J
    K = A.m_K
    phi = lambda x: max(A.phi(x), - B.phi(x)) # Difference => max(A(x), - B(x))
    C = Grid(min_coord, max_coord, I, J, K, phi=phi)
    return C

def translate(A, t):
    min_coord = A.m_min_coord - np.array(t)
    max_coord = A.m_max_coord - np.array(t)
    phi = lambda x: A.phi(x - np.array(t))
    C = Grid(min_coord, max_coord, A.m_I, A.m_J, A.m_K, phi=phi)
    return C

def scale(A, s):
    min_coord = A.m_min_coord
    max_coord = A.m_max_coord
    phi = lambda x: s * A.phi(np.array(x) / s)
    C = Grid(min_coord, max_coord, A.m_I, A.m_J, A.m_K, phi=phi)
    return C

def rotate(A, R):
    min_coord = A.m_min_coord
    max_coord = A.m_max_coord
    phi = lambda x: A.phi(np.dot(R.T, np.array(x)))
    C = Grid(min_coord, max_coord, A.m_I, A.m_J, A.m_K, phi=phi)
    return C

def erosion(A, delta):
    min_coord = A.m_min_coord
    max_coord = A.m_max_coord
    phi = lambda x: A.phi(x) - delta
    C = Grid(min_coord, max_coord, A.m_I, A.m_J, A.m_K, phi=phi)
    return C

def dilation(A, delta):
    return erosion(A, -delta)

def opening(A, delta):
    return dilation(erosion(A, delta), delta)
