# Displaying Faces on flo

The display_faces.ino file is used to read in serial data from an attached
computer and display faces to the LED boards. 

The send_faces.py file is used to send a desired face from the computer to the
robot over the serial connection. It takes in a json file which holds face
definitions. There is an example JSON file at faces_demo.json. The eye and
mouth patterns are defined individually and then put together into faces. 
Indexing is as if you were looking at the matrix dead on. Valid values for the
`on` matrix are 0 and 1. Extra key value pairs can be added where deemed useful.
The on matrix is required for eyes and faces. The demo file shows a description
as a possible added field, but you can create any others which you think useful.

For eyes the idea is to have a mood specified for a face, then have an eye for
each direction, which can be specified at a higher level. 

You can run the read_faces file to test that it reads the way you want. 
You can change the mood of the face and the eye direction at the top of the 
file. 