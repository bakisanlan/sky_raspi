#from get_wp_cmd import get_wp_cmd
import math

# Define a function to convert latitude, longitude, and altitude to Cartesian coordinates
def geodetic_to_NED(ref_location, target_location):
    # Convert degrees to radians
    lat0 = ref_location.lat
    lat0 = math.radians(lat0)
    lon0 = ref_location.lon
    lon0 = math.radians(lon0)
    h0 = ref_location.alt
    
    lat = target_location.lat
    lat = math.radians(lat)
    lon = target_location.lon
    lon = math.radians(lon)
    h = target_location.alt

    # WGS84 ellipsoid parameters
    a = 6378137.0
    f = 1/298.257223563
    b = a*(1-f)
    e = math.sqrt((a**2-b**2)/a**2)

    # Calculate ECEF coordinates of reference point
    N = a/math.sqrt(1-e**2*math.sin(lat0)**2)
    x0 = (N+h0)*math.cos(lat0)*math.cos(lon0)
    y0 = (N+h0)*math.cos(lat0)*math.sin(lon0)
    z0 = ((1-e**2)*N+h0)*math.sin(lat0)

    # Calculate ECEF coordinates of point to convert
    N = a/math.sqrt(1-e**2*math.sin(lat)**2)
    x = (N+h)*math.cos(lat)*math.cos(lon)
    y = (N+h)*math.cos(lat)*math.sin(lon)
    z = ((1-e**2)*N+h)*math.sin(lat)

    # Calculate NED coordinates
    dx = x-x0
    dy = y-y0
    dz = z-z0
    north = -math.sin(lat0)*math.cos(lon0)*dx \
            -math.sin(lat0)*math.sin(lon0)*dy \
            +math.cos(lat0)*dz
    east = -math.sin(lon0)*dx \
           +math.cos(lon0)*dy
    down = -math.cos(lat0)*math.cos(lon0)*dx \
           -math.cos(lat0)*math.sin(lon0)*dy \
           -math.sin(lat0)*dz

    return (north, east, down)