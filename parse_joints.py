import xml.etree.ElementTree as ET
tree = ET.parse("src/ridgeback_vision_detection/urdf/ridgeback.urdf.xacro")
for j in tree.iter('joint'):
    print(j.get('name'))
