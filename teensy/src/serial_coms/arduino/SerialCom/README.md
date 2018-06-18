# Arduino Serial Communications

## Handling arbitrary length messages
In order to buffer messages, we need a resizing array. This could become dangerous!! We will initialize with an array of size 1 byte. When that size is exceeded (probably right away) we will allocate a new array of the necessary size * an oversize factor (to keep from constantly needing to make new arrays) which will default to 1.25. 

## Message and data arrays
There are two arrays being used, the message array and the data array. Once a complete message is present in the message array, its data is copied to the data array.

## Acting on the data
When the data array is populated, an event handler, defined at initialization will be called. The data array will always contain the newest 

## Preventing blocking
The program will never completely block, but since reading from the serial buffer can be time consuming, it is possible to set a maximum number of bits to read per loop. 

## Running 
In order to allow the serial communications to run, the update function must be called every loop by the main loop. 

## Why not run async with interrupts?
We don't want to use interrupts because we want whatever is acting on the data to be able to do its thing before we take in any more data. The danger of this is that the serial buffer could fill up before we get to the data. 

## Error Handling
I don't like the way I am doing error handling now. I am open to suggestions

## Installing package
Either 1) put the package in your arduino library folder or 2) set a symlink from your arduino library folder to the SerialCom folder. You can find your arduino library folder by looking inside the folder called "Sketchbook location" which is noted in the preferences of the Arduino IDE.