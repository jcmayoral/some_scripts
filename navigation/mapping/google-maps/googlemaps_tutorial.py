import googlemaps
from datetime import datetime

gmaps = googlemaps.Client(key='AIzaSyC5D42RbkvsG7Kd2CBwrGdHDIZlMwpIJrQ')

# Geocoding an address
geocode_result = gmaps.geocode('Hochschule Bonn Rhein Sieg')
#print (geocode_result)

# Look up an address with reverse geocoding
reverse_geocode_result = gmaps.reverse_geocode((40.714224, -73.961452))

# Request directions via public transit
now = datetime.now()
directions_result = gmaps.directions("Hochschule Bonn Rhein Sieg",
                                     "HUMA, Sankt Augustin",
                                     mode="transit",
                                     departure_time=now)


for i in directions_result:
    for key,value in i["legs"][0].iteritems():
        print(key)
        print(value)
