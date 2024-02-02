import math

def NED_to_lat_lon(ref_location, north, east):
    # convert north and east values to latitude and longitude offsets
    earth_radius = 6378137  # radius of the earth in meters
    lat_offset = north / earth_radius * (180 / math.pi)
    lon_offset = east / earth_radius * (180 / math.pi) / math.cos(ref_location.lat * math.pi / 180)

    # calculate new latitude and longitude
    obj_lat = ref_location.lat + lat_offset
    obj_lon = ref_location.lon + lon_offset

    return (obj_lat,obj_lon)    
