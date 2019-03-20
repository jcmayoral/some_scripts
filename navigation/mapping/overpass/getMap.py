import overpass

api = overpass.API()

#print api.get('node["name"]="Oslo"', responseformat='xml')


mapquery = overpass.MapQuery(59.62, 10.7906 , 59.68, 10.79563)
requested_map = api.get(mapquery,responseformat='xml')

f =  open("mymap.xml", "wb")
f.write(requested_map.encode('ascii', 'ignore').decode('ascii'))
f.close()
