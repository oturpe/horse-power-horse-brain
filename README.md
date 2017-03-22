# Horse Power horse brain
Controller for horse themed bio-robotic sculpture

## Programming

Brain of the brain is [Arduino Nano][nano] microcontroller. The programming environment is Arduino. It can be probably uploaded using [Arduino ide][ide], however during development the following command line commands were used exclusively:

[nano]: https://www.arduino.cc/en/Main/arduinoBoardNano
[ide]: https://www.arduino.cc/en/main/software

Build and upload:

```
arduino --build "`pdw`/horse-brain.ino"
```

Connect with serial monitor (through [Ino][ino]):

```
ino serial
```

[ino]: http://inotool.org/

## Hardware

Hardware design can be found in `hardware` folder. Is it created with [KiCad][kicad]. There is no printed circuit board, the board was hand soldered and the schematic was afterwards created for documentation purposes.

[kicad]: http://kicad-pcb.org/
