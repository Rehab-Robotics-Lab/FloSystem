# flo_head

## Dependencies:
- https://github.com/adafruit/Adafruit_LED_Backpack
- https://github.com/adafruit/Adafruit-GFX-Library 

## Setting up premade faces
See the [demo folder](/demo/)

## Communications
Messages will have the following format:

| item to be updated | binary array of the update |
|:------------------:|:--------------------------:|
| 0 - Mouth          | 16 bytes                   |
| 1 - Left Eye       | 8 bytes                    |
| 2 - Right Eye      | 8 bytes                    |

