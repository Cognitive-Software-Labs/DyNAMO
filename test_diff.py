import xml.etree.ElementTree as ET
tree = ET.parse("src/ridgeback_vision_detection/urdf/ridgeback.gazebo")
root = tree.getroot()
print(ET.tostring(root).decode())
