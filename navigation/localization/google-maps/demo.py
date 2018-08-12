#Source https://pypi.python.org/pypi/osmapi

import osmapi

api = osmapi.OsmApi()
print (api.NodeGet(123))

api = osmapi.OsmApi(username = "JoseMayoral", password = "q2p0w3o9e4i8")
print (api.NodeGet(123))

#api = osmapi.OsmApi(username = "you", passwordfile = "/etc/mypasswords")
#api = osmapi.OsmApi(passwordfile = "/etc/mypasswords") # if only the passwordfile is specified, the credentials on the first line of the file will be used


api.ChangesetCreate({"comment": "My first test"})
print (api.NodeCreate({"lon":1, "lat":1, "tag": {}}))
# {u'changeset': 532907, u'lon': 1, u'version': 1, u'lat': 1, u'tag': {}, u'id': 164684}
api.ChangesetClose()
