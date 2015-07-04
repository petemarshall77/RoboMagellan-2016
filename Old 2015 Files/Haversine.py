import math

def degrees_to_radians(angle):
	return angle*math.pi/180.0

def radians_to_degrees(radians):
	return radians*180/math.pi

def get_distance_and_bearing(from_lat,from_long,to_lat,to_long):
	from_lat=degrees_to_radians(from_lat)
	from_long=degrees_to_radians(from_long)
	to_lat=degrees_to_radians(to_lat)
	to_long=degrees_to_radians(to_long)
	delta_lat=to_lat-from_lat
	delta_long=to_long-from_long
	
	a=math.sin(delta_lat/2)*math.sin(delta_lat/2)+math.cos(from_lat)*math.cos(to_lat)*math.sin(delta_long/2)*math.sin(delta_long/2)
	c=2*(math.atan2(math.sqrt(a),math.sqrt(1-a)))
	distance=6371000*c
	
	y=math.sin(from_long-to_long)*math.cos(to_lat)
	x=math.cos(from_lat)*math.sin(to_lat)-math.sin(from_lat)*math.cos(to_lat)*math.cos(from_long-to_long)
	bearing=(radians_to_degrees(math.atan2(y,x))+360)%360
	
	return (distance,bearing)

print get_distance_and_bearing(33.7780555556,118.4188888889,33.7805555556,118.4191666667)
