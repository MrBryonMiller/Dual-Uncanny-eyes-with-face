# Dual-Uncanny-eyes-with-face
Dual Uncanny eyes with background face is based on AdaFruit's Uncanny eyes project.  This derivative would not be possible without their work.  This program was designed and tested with their product 3651 "Adafruit TFT FeatherWing - 3.5" 480x320 Touchscreen for Feathers" plus a Feather M0.  Initial testing included a M4 Feather Express but M4 testing did not take place on the final version.  

Other than plugging the Feather into the Display, the only wiring I use is a pulldown resistor (1k to ground) on the Display's backlight control signal,  with further wiring to pin 11 of the Feather. This wiring is optional, but I like it because it keeps the screen blank upon power-up until the face is drawn.  The only other thing needed is a micro SD card, formated and with the file "clown.bmp" included in the card's root directory.  You can find this file in the "SDCard" directory

The original project has many options settable in "config.h".  I tried to maintain compatibility with these options but in several cases I felt I needed to simplify; so the final result may, or may not be compatible with some or all of the options.  I only tested in my as-delivered configuration.

