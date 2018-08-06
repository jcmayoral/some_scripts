import os
from dynamic_reconfigure.client import Client
import rospy
import re

rospy.init_node("dynamic_reconfigure_planners")

dyn_client = Client("move_base", None)


plugins = os.popen("rospack plugins --attrib=plugin nav_core").read()
plugins = plugins.splitlines()
plugin_name_type = list()

for i in plugins:
    j = i.split()
    c_file = open(j[1],"r")
    for line in c_file:
        index_name = line.find("name")
        index_type = line.find("type")
        base_class_index = line.find("base_class_type")

        if index_name > 0:
            name = re.findall(r'"(.*?)"', line[index_name:])[0]
            if index_type>0:
                plugin_type = re.findall(r'"(.*?)"', line[index_type:])[0]
                if base_class_index > 0:
                    plugin_base_type = re.findall(r'"(.*?)"', line[base_class_index:])[0]
                    plugin_name_type.append([name, plugin_type, plugin_base_type])

for n_t in plugin_name_type:
    if n_t[2] == "nav_core::BaseGlobalPlanner":
        print "name ", n_t[0] , " type " , n_t[1], "Base Type" , n_t[2]
        dyn_client.update_configuration({"global_planner": n_t[0]})
        break

print "finish"
