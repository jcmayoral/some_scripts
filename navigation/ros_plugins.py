import os
from dynamic_reconfigure.client import Client
import rospy
import rosparam
import re
import rospkg

def delete_old_params(complete_namespace):
    rospy.loginfo("Deleting old params")
    if not rospy.has_param(complete_namespace):
        rospy.logwarn("Parameters not set")
        return

    old_parameters = rosparam.get_param(complete_namespace)
    for param in old_parameters:
        try:
            rosparam.delete_param(complete_namespace+'/'+param)
        except:
            rospy.logwarn("param not found %s", complete_namespace+'/'+param)

def load_new_params(filepath):
    rospy.loginfo("Loading new params")
    new_config_file = rosparam.load_file(filepath)

    for params,ns in new_config_file:
        rosparam.upload_params(navigation_server + ns,params)

rospy.init_node("dynamic_reconfigure_planners")

navigation_server = "/move_base/"
config_package = "config_pkg"

dyn_client = Client(navigation_server, None)

current_global_planner = rosparam.get_param(navigation_server + "base_global_planner")

try:
    current_global_planner_ns = current_global_planner.split('/')[0]
    current_global_planner_name = current_global_planner.split('/')[1]
except:
    current_global_planner_ns = ""
    current_global_planner_name = current_global_planner.split('/')[0]

current_local_planner = rosparam.get_param(navigation_server + "base_local_planner")

try:
    current_local_planner_ns = current_local_planner.split('/')[0]
    current_local_planner_name = current_local_planner.split('/')[1]
except:
    current_local_planner_ns = ""
    current_local_planner_name = current_local_planner.split('/')[0]


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
new_config["base_global_planner"] = "Error"
new_config["base_local_planner"] = "Error"

print "Select Your Global Planner"

for i in range(len(available_global_planners)):
    print "Available GP ", i , available_global_planners[i]

new_config["base_global_planner"] = available_global_planners[int(raw_input('Choose a number: '))]

print "Select Your Local Planner"

for i in range(len(available_local_planners)):
    print "Available LP ", i , available_local_planners[i]

new_config["base_local_planner"] = available_local_planners[int(raw_input('Choose a number: '))]

is_to_delete = raw_input("Do you want to delete old parameters? y/n")

if is_to_delete is "y":
    delete_old_params(navigation_server + current_global_planner_name)
    delete_old_params(navigation_server + current_local_planner_name)

rospack = rospkg.RosPack()
try:
    config_path = rospack.get_path(config_package)
    load_new_params(config_path+'/'+ new_config["base_global_planner"].split('/')[0] + '.yaml')
    load_new_params(config_path+'/'+ new_config["base_local_planner"].split('/')[0] + '.yaml')
except:
    rospy.logerr("Config package is not found")


dyn_client.update_configuration(new_config)

rospy.loginfo("Planners have been updated")
