#From mongodb_store



def fill_message(message, document):
    """
    Fill a ROS message from a dictionary, assuming the slots of the message are keys in the dictionary.

    :Args:
        | message (ROS message): An instance of a ROS message that will be filled in
        | document (dict): A dicionary containing all of the message attributes

    Example:

    >>> from geometry_msgs.msg import Pose
    >>> d = dcu.msg_to_document(Pose())
    >>> d['position']['x']=27.0
    >>> new_pose = Pose(
    >>> fill_message(new_pose, d)
    >>>  new_pose
    position:
      x: 27.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
    """
    for slot, slot_type in zip(message.__slots__,
                               getattr(message,"_slot_types",[""]*len(message.__slots__))):

        # This check is required since objects returned with projection queries can have absent keys
        if slot in document.keys():
            value = document[slot]
        # fill internal structures if value is a dictionary itself
            if isinstance(value, dict):
                fill_message(getattr(message, slot), value)
            elif isinstance(value, list) and slot_type.find("/")!=-1:
            # if its a list and the type is some message (contains a "/")
                lst=[]
            # Remove [] from message type ([:-2])
                msg_type = type_to_class_string(slot_type[:-2])
                msg_class = load_class(msg_type)
                for i in value:
                    msg = msg_class()
                    fill_message(msg, i)
                    lst.append(msg)
                    setattr(message, slot, lst)
            else:
                if isinstance(value, unicode):
                    setattr(message, slot, value.encode('utf-8'))
                else:
                    setattr(message, slot, value)

def dictionary_to_message(dictionary, cls):
    """
    Create a ROS message from the given dictionary, using fill_message.

    :Args:
        | dictionary (dict): A dictionary containing all of the atributes of the message
        | cls (class): The python class of the ROS message type being reconstructed.
    :Returns:
        An instance of cls with the attributes filled.


    Example:

    >>> from geometry_msgs.msg import Pose
    >>> d = {'orientation': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
       'position': {'x': 27.0, 'y': 0.0, 'z': 0.0}}
    >>> dictionary_to_message(d, Pose)
    position:
      x: 27.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
    """
    message = cls()

    fill_message(message, dictionary)

    return message
