# The Midi Levitator
Controlling objects floating in space with music, using a midi interface.

 ![Floating Balls](FloatingBalls.jpg)![Floating Balls](FloatingBalls.jpg)![Floating Balls](FloatingBalls.jpg)![Floating Balls](FloatingBalls.jpg)


A midi keyboard connected to [Ableton Live](https://en.wikipedia.org/wiki/Ableton_Live) outputs serial commands using [Max software](https://en.wikipedia.org/wiki/Max_(software)) to an ESP32 microcontroller that is controlling a [SonicSurface](https://github.com/upnalab/SonicSurface) acoustic levitator to float objects in space that respond to the keyboard.

Video: https://youtu.be/8fpSVYPssQY

# Contents of this Repository
- [ESP32Code](Firmware/ESP32%20controller) - Software to run on an ESP32 microcontroller that controls the [SonicSurface](https://github.com/upnalab/SonicSurface).
- [Max Software](MaxSoftware) - [Software](https://en.wikipedia.org/wiki/Max_(software)) that runs on [Ableton Live](https://en.wikipedia.org/wiki/Ableton_Live) which accepts midi input and sends commands via serial port to the ESP32.
- Project Report- Coming soon!


