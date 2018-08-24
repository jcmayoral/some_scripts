import os
from dynamic_reconfigure.client import Client
import rospy
import rosparam
import re

rospy.init_node("dynamic_reconfigure_planners")

dyn_client = Client("/navigation/move_base_flex", None)


plugins = os.popen("rospack plugins --attrib=plugin nav_core").read()
plugins = plugins.splitlines()
plugin_name_type = list()

for i in plugins:
    j = i.split()
    c_file = open(j[1],"r")
    flags = [False,False,False]
    for line in c_file:
        index_name = line.find("name")
        index_type = line.find("type")
        base_class_index = line.find("base_class_type")

        if index_name > 0 and not flags[0]:
            name = re.findall(r'"(.*?)"', line[index_name:])[0]
            flags[0] = True
        if index_type>0 and not flags[1]:
            plugin_type = re.findall(r'"(.*?)"', line[index_type:])[0]
            flags[1] = True
        if base_class_index > 0 and not flags[2]:
            plugin_base_type = re.findall(r'"(.*?)"', line[base_class_index:])[0]
            flags[2] = True
        
        if all(flags):
            plugin_name_type.append([name, plugin_type, plugin_base_type])
            flags = [False,False,False]

available_global_planners = list()
available_local_planners = list()

for n_t in plugin_name_type:
    print n_t[0], n_t[2]
    if n_t[2] == "nav_core::BaseGlobalPlanner":
        available_global_planners.append(n_t[0])

    if n_t[2] == "nav_core::BaseLocalPlanner":
        available_local_planners.append(n_t[0])


new_config = dict()
new_config["global_planner"] = "Error"
new_config["local_planner"] = "Error"

print "Select Your Global Planner"

for i in range(len(available_global_planners)):
    print "Available GP ", i , available_global_planners[i]

new_config["global_planner"] = available_global_planners[int(raw_input('Choose a number: '))]

print "Select Your Local Planner"

for i in range(len(available_local_planners)):
    print "Available LP ", i , available_local_planners[i]

new_config["local_planner"] = available_local_planners[int(raw_input('Choose a number: '))]

print new_config


print "DELETING OLD PARAMS"

ns = '/ns'

old_parameters = rosparam.get_param(ns)

for param in b:
    rosparam.delete_param(ns+param)

print "LOADING NEW PARAMS"

new_config_file = rosparam.load_file('/home/banos/Downloads/cheat.yaml')

for params,ns in new_config_file:
    rosparam.upload_params(ns,params)


dyn_client.update_configuration(new_config)
#break

print "finish"
