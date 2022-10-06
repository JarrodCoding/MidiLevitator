# ESP32 Software
The Sonic surface is controlled using an ESP32 microcontroller running the Arduino code in this directory.

## ESP Commands

Commands, that are case sensitive, are entered through a serial terminal to the ESP32. They are:-

- reset 

Set the focal point to the centre of the SonicSurface cube. This is the place to launch objects into the levitation field.

- test 

This starts a test pattern that will run forever. The pattern is a square spiral that grows from the centre in 5mm increments until the the side length exceeds 65mm. The first spiral starts at the current height and increases in 5mm increments until it reaches 185 mm at which point it spirals in decreasing heights until it reaches 55 mm. The object is moved in 1mm increments 40 times per second. The levitated object traverses a large region which is good for testing trapping stability throughout.

- bpm=

Set the beats per minute up to 70. Levitated object moves at that beat.

- stop 

Stops objects moving to the beat.

- mode=

0, 1 or 2. This switches between control modes. In mode 0 three keyboard dials will move an object in the x, y or z direction.
In mode 1, just the keyboard will move objects in the x direction. In mode 2 the keyboard will move objects in the x direction and two dials will move objects in the the y and z direction.

## Other ESP32 Commands

All following commands must have a trailing space.

- on

Switch all transducers on.
- off

Switch all transducers off.
- top

Switch all transducers in top array on.

- bottom

Switch all transducers in bottom array on.

- focus= x y z 

Set the focus point. Positions are in metres and seperated by a space. 0, half array height, 0 will be at the centre of the array.

- focusMulti= x y z x y z ...

Set multiple focus points. Positions are in metres and seperated by a space. Up to 4 points can be specified and the trapping force is lower than for single focus points.

- itersIBP= 

Set IBP iterations. 

- focusIBP= x y z x y z x y z x y z 

Multipoint with IBP iterations. Slower than focusMulti.

- phases=

Send whatever phases are currently defined to the SonicSurface.
 
- switch

Switch the phases between the top and bottom array.

- version 

Display version.
