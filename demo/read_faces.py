import json
from pprint import pprint

filename = "faces_demo.json"
mood = "sad"
eye_direction = "left"


json_data=open(filename)
faces = json.load(json_data)

mouth = faces["mouths"][faces["faces"][mood]["mouth"]]["on"]
l_eye = faces["eyes"][faces["faces"][mood]["l-eye"]][eye_direction]["on"]
r_eye = faces["eyes"][faces["faces"][mood]["r-eye"]][eye_direction]["on"]

print("mouth: ")
pprint(mouth)

print("left eye: ")
pprint(l_eye)

print("right eye: ")
pprint(r_eye)