"""
 Reading mesoNH netCDF files and providing an interface to read chunks
of data and interpolate points.

 TODO : test with large netCDF files, add mapping of variables as an argument.
"""

from __future__ import division, print_function, absolute_import
from netCDF4 import MFDataset, Dataset
import numpy as np
from scipy.interpolate import RegularGridInterpolator


def find_lt(a, x):
    'Find rightmost value less than x'
    return np.searchsorted(a, x, side='left')-1


def find_le(a, x):
    'Find rightmost value less than or equal to x'
    return np.searchsorted(a, x, side='right')-1


def find_gt(a, x):
    'Find leftmost value greater than x'
    return np.searchsorted(a, x, side='right')


def find_ge(a, x):
    'Find leftmost item greater than or equal to x'
    return np.searchsorted(a, x, side='left')

WX = 'UT'  # Wind x,y and z composants
WY = 'VT'
WZ = 'WT'
PTEMP = 'THT'  # potential temperature
RVAP = 'RVT'  # vapor mixing ratio
LWATER = 'RCT'  # liquid water
ZSCALE = 'VLEV'  # vertical level (height) scale
XSCALE = 'S_N_direction'  # south -> north axis scale
YSCALE = 'W_E_direction'  # west -> east axis scale
TIME = 'time'  # time dimension
X = 2
Y = 3
Z = 1
T = 0


class MesoNHAtmosphere:
    """
     Interpolation in space-time from mesoNH data over a grid.

     The functions valued on the grid are assumed to be periodic in the
     x and y axis (south->north and west->east) :
     f(x)=f(x+xmax-xmin) where [xmin,xmax] are the bounds on x
     of the mesoNH grid.

         Clipping is done on the z (height) and time axis :
     f(z)=f(min(max(z,zmin),zmax)) where [zmin,zmax] are the bound on z
     of the mesoNH grid.

     Two interpolation methods are currently supported : nearest neighbour
     and linear.

     Uses RegularGridInterpolator from scipy package.

     """
    data = []

    interpolator = None
    boundsmax = None
    boundsmin = None

    def __init__(self, files, tstep, tinit=0):
        if np.isscalar(files):
            self.data = Dataset(files)
        else:
            self.data = MFDataset(files)
        pz = self.data.variables[ZSCALE][:, 0, 0]
        px = self.data.variables[XSCALE]
        py = self.data.variables[YSCALE]
        pt = np.array([tinit + i * tstep for i in
                       range(self.data.variables[TIME].shape[0])])
        self.boundsmax = [pt[-1], pz[-1], px[-1], py[-1]]
        self.boundsmin = [pt[0], pz[0], px[0], py[0]]

        self.grid_coordinates = (pt, pz, px, py)
        self.grid_shape = (len(pt), len(pz), len(px), len(py))

    def get_points(self, points, var, method='nearest'):
        """ Get value of variable on points
   
        Arguments:
        points: a ndarray containing the point coordinates on the last
        dimension
        var: the name of the variable in the mesoNH file(s)
        method: 'nearest' and 'linear' interpolation are currently supported
        """
        points = np.array(points)
        p = self._apply_bounds(points)
        caxes = tuple(range(p.ndim-1))
        bounds = zip(p.min(axis=caxes), p.max(axis=caxes))
        interpolator = self._get_interpolator(bounds, var, method)
        return interpolator(p, method).squeeze()

    def _get_interpolator(self, bounds, var, method="nearest"):
        slice_indexes, coordinates = self._slicyfy(bounds)
        values = self._get_var_values(var, slice_indexes)
        ip = RegularGridInterpolator(coordinates, values, method)
        return ip

    def _slicyfy(self, bounds):
        slice_indexes = ()
        coordinates = ()
        for d, b in enumerate(bounds):
            dslice = slice(find_le(self.grid_coordinates[d], b[0]),
                           find_gt(self.grid_coordinates[d], b[1])+1)
            slice_indexes += dslice,
            coordinates += self.grid_coordinates[d][dslice],

        return slice_indexes, coordinates

    def _get_var_values(self, var, idx=Ellipsis):
        return self.data[var][idx]

    def _apply_bounds(self, point):
        x = point[..., X]
        y = point[..., Y]
        z = point[..., Z]
        t = point[..., T]

        p = np.ndarray(point.shape)
        p[..., T] = np.clip(t, self.boundsmin[T], self.boundsmax[T])
        p[..., Z] = np.clip(z, self.boundsmin[Z], self.boundsmax[Z])
        p[..., X] = (x-self.boundsmin[X]) % (self.boundsmax[X]-self.boundsmin[X])\
            + self.boundsmin[X]
        p[..., Y] = (y-self.boundsmin[Y]) % (self.boundsmax[Y]-self.boundsmin[Y])\
            + self.boundsmin[Y]

        return p

    def get_wind(self, points, method='nearest'):
        """Convenience method for getting 3D wind. See get_points."""
        return np.array((self.get_points(points, WX, method),
                         self.get_points(points, WY, method),
                         self.get_points(points, WZ, method)
                         ))
