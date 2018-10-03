//--------------------------------------------------------------------------
// Dual Uncanny eyes with background face is based on AdaFruit's Uncanny eyes 
//  project.  This derivative would not be possible without their work.
//  This program was designed and tested with their product 3651
//  "Adafruit TFT FeatherWing - 3.5" 480x320 Touchscreen for Feathers"
//  plus a Feather M0.  Initial testing included a M4 Feather Express but this
//  testing did not take place on the final version.  
//
// Other than plugging the Feather into the Display, the only wiring I use is a 
//  pulldown resistor (1k to ground) on the Display's backlight control signal, 
//  with further wiring to pin 11 of the Feather.
//  This wiring is optional, but I like it because it keeps the screen blank
//  upon power-up until the face is drawn.  The only other thing needed is a 
//  micro SD card, formated and with the file "clown.bmp" included in the card's 
//  root directory.  
//
// The original project has many options settable in "config.h".  I tried to
//  maintain compatibility with these options but in several cases I felt I
//  needed to simplify; so the final result may, or may not be compatible with
//  some or all of the options.  I only tested in my as-delivered configuration.
//
// The following is the original file header...
//--------------------------------------------------------------------------
// Uncanny eyes for Adafruit 1.5" OLED (product #1431) or 1.44" TFT LCD
// (#2088).  Works on PJRC Teensy 3.x and on Adafruit M0 and M4 boards
// (Feather, Metro, etc.).  This code uses features specific to these
// boards and WILL NOT work on normal Arduino or other boards!
//
// SEE FILE "config.h" FOR MOST CONFIGURATION (graphics, pins, display type,
// etc).  Probably won't need to edit THIS file unless you're doing some
// extremely custom modifications.
//
// Adafruit invests time and resources providing this open source code,
// please support Adafruit and open-source hardware by purchasing products
// from Adafruit!
//
// Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.
// MIT license.  SPI FIFO insight from Paul Stoffregen's ILI9341_t3 library.
// Inspired by David Boccabella's (Marcwolf) hybrid servo/OLED eye concept.
//--------------------------------------------------------------------------

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ZeroDMA.h>
#include <SD.h>
#define SD_CS 5

typedef struct {        // Struct is defined before including config.h --
  int8_t  select;       // pin numbers for each eye's screen select line
  int8_t  wink;         // and wink button (or -1 if none) specified there,
  uint8_t rotation;     // also display rotation.
} eyeInfo_t;

#include "config.h"     // ****** CONFIGURATION IS DONE IN HERE ******
#include "graphics/clowneye.h";
typedef Adafruit_HX8357  displayType; // Using TFT display(s)

// A simple state machine is used to control eye blinks/winks:
#define NOBLINK 0       // Not currently engaged in a blink
#define ENBLINK 1       // Eyelid is currently closing
#define DEBLINK 2       // Eyelid is currently opening
typedef struct {
  uint8_t  state;       // NOBLINK/ENBLINK/DEBLINK
  uint32_t duration;    // Duration of blink state (micros)
  uint32_t startTime;   // Time (micros) of last state change
} eyeBlink;

#define NUM_EYES (sizeof eyeInfo / sizeof eyeInfo[0]) // config.h pin list

struct {                // One-per-eye structure
  displayType *display; // -> OLED/TFT object
  eyeBlink     blink;   // Current blink/wink state
} eye[NUM_EYES];

displayType *tft;

// SAMD boards use DMA (Teensy uses SPI FIFO instead):
// Two single-line 128-pixel buffers (16bpp) are used for DMA.
// Though you'd think fewer larger transfers would improve speed,
// multi-line buffering made no appreciable difference.
uint16_t          dmaBuf[2][128];
uint8_t           dmaIdx = 0; // Active DMA buffer # (alternate fill/send)
Adafruit_ZeroDMA  dma;
DmacDescriptor   *descriptor;

// DMA transfer-in-progress indicator and callback
static volatile bool dma_busy = false;
static void dma_callback(Adafruit_ZeroDMA *dma) { dma_busy = false; }

//const uint8_t mirrorTFT[]  = { 0x80, 0x20, 0x20, 0xE0 }; // Mirror+rotate
uint8_t MADCTL[] = {0x20,0x20,0xE0};
uint16_t XOFFSET[] = {283,70,213};
uint16_t YOFFSET[] = {66,66,40};
bool eyeFreeze;

/* 
I put this here so Notepad++ would recognize this as a Function
void setMADCTL(void){}
*/
void setMADCTL(uint8_t f)	// f=feature 0 first eye,1 second eye, 2 face
{
//  const uint8_t mirrorTFT[]  = { 0x88, 0x28, 0x48, 0xE8 }; // Mirror+rotate
//  const uint8_t mirrorTFT[]  = { 0x80, 0x20, 0xA0, 0xE0 }; // Mirror+rotate
digitalWrite(eyeInfo[0].select, LOW);
digitalWrite(DISPLAY_DC, LOW);
SPI.transfer(HX8357_MADCTL); // Current TFT lib
digitalWrite(DISPLAY_DC, HIGH);
//SPI.transfer(mirrorTFT[eyeInfo[e].rotation & 3]);
SPI.transfer(MADCTL[f]);
delay(1);
digitalWrite(eyeInfo[0].select , HIGH);
}


uint32_t startTime;  // For FPS indicator
uint32_t eyeChange = 0;

// INITIALIZATION -- runs once at startup ----------------------------------

void setup(void) 
{

pinMode(11,OUTPUT);	// screen backlight

  uint8_t e; // Eye index, 0 to NUM_EYES-1

  Serial.begin(115200);
int waitcnt = 0;
//while(!Serial && (waitcnt++ < 50))  // wait for 5 secs (development only.
while(!Serial && (waitcnt++ < 1))  // wait 100 ms.
     delay(100);
Serial.println("HX8357D Uncanny Eyes!"); 
randomSeed(analogRead(A3)); // Seed random() from floating analog input


  // Initialize eye objects based on eyeInfo list in config.h:
for(e=0; e<NUM_EYES; e++) 
	{
	eye[e].blink.state = NOBLINK;
    // Also set up an individual eye-wink pin if defined:
	if(eyeInfo[e].wink >= 0) 
		pinMode(eyeInfo[e].wink, INPUT_PULLUP);
  }
tft = new displayType(eyeInfo[0].select, DISPLAY_DC, -1);
pinMode(eyeInfo[0].select, OUTPUT);
digitalWrite(eyeInfo[0].select, HIGH); // Deselect
#if defined(BLINK_PIN) && (BLINK_PIN >= 0)
  pinMode(BLINK_PIN, INPUT_PULLUP); // Ditto for all-eyes blink pin
Serial.println("BLINK_PIN"); 
#endif


// After all-displays reset, now call init/begin func 
tft->begin();
tft->setRotation(1);

tft->fillScreen(HX8357_BLACK);
setMADCTL(1);

#ifdef ARDUINO_ARCH_SAMD
  // Set up SPI DMA on SAMD boards:
  int                dmac_id;
  volatile uint32_t *data_reg;
  if(&PERIPH_SPI == &sercom0) {
    dmac_id  = SERCOM0_DMAC_ID_TX;
    data_reg = &SERCOM0->SPI.DATA.reg;
    Serial.println("SERCOM0");
#if defined SERCOM1
  } else if(&PERIPH_SPI == &sercom1) {
    dmac_id  = SERCOM1_DMAC_ID_TX;
    data_reg = &SERCOM1->SPI.DATA.reg;
    Serial.println("SERCOM1");
#endif
#if defined SERCOM2
  } else if(&PERIPH_SPI == &sercom2) {
    dmac_id  = SERCOM2_DMAC_ID_TX;
    data_reg = &SERCOM2->SPI.DATA.reg;
    Serial.println("SERCOM2");
#endif
#if defined SERCOM3
  } else if(&PERIPH_SPI == &sercom3) {
    dmac_id  = SERCOM3_DMAC_ID_TX;
    data_reg = &SERCOM3->SPI.DATA.reg;
    Serial.println("SERCOM3");
#endif
#if defined SERCOM4
  } else if(&PERIPH_SPI == &sercom4) {
    dmac_id  = SERCOM4_DMAC_ID_TX;
    data_reg = &SERCOM4->SPI.DATA.reg;
    Serial.println("SERCOM4");
#endif
#if defined SERCOM5
  } else if(&PERIPH_SPI == &sercom5) {
    dmac_id  = SERCOM5_DMAC_ID_TX;
    data_reg = &SERCOM5->SPI.DATA.reg;
    Serial.println("SERCOM5");
#endif
  }

dma.allocate();
dma.setTrigger(dmac_id);
dma.setAction(DMA_TRIGGER_ACTON_BEAT);
descriptor = dma.addDescriptor(
				NULL,               // move data
				(void *)data_reg,   // to here
				sizeof dmaBuf[0],   // this many...
				DMA_BEAT_SIZE_BYTE, // bytes/hword/words
				true,               // increment source addr?
				false);             // increment dest addr?
dma.setCallback(dma_callback);

#endif // End SAMD-specific SPI DMA init

// if you want to see face drawn, turn on screen backlight now
// otherwise comment out next line and just have working face
// pop up, albeit after several seconds
digitalWrite(11,HIGH);	

drawFace();
digitalWrite(11,HIGH);	// turn on screen backlight

startTime = millis(); // For frame-rate calculation
}

SPISettings settings(SPI_FREQ, MSBFIRST, SPI_MODE0);


// FACE-RENDERING FUNCTION -------------------------------------------------

void drawFace()
{
Serial.print("Initializing SD card...");
if (!SD.begin(SD_CS)) {
Serial.println("failed!");
}
Serial.println("OK!");
digitalWrite(11,HIGH);	// turn on screen backlight

bmpDraw("clown.bmp", 0, 0);
//bmpDraw("face.bmp", 0, 0);

}

// EYE-RENDERING FUNCTION --------------------------------------------------

void drawEye( // Renders one eye.  Inputs must be pre-clipped & valid.
		  uint8_t  e,       // Eye array index; 0 or 1 for left/right
		  uint32_t iScale,  // Scale factor for iris
		  uint8_t  scleraX, // First pixel X offset into sclera image
		  uint8_t  scleraY, // First pixel Y offset into sclera image
		  uint8_t  uT,      // Upper eyelid threshold value
		  uint8_t  lT) 
{    // Lower eyelid threshold value
	
uint8_t  screenX, screenY, scleraXsave;
int16_t  irisX, irisY;
uint16_t p, a;
uint32_t d;

setMADCTL(e);
// Set up raw pixel dump to entire screen.  Although such writes can wrap
// around automatically from end of rect back to beginning, the region is
// reset on each frame here in case of an SPI glitch.
SPI.beginTransaction(settings);
digitalWrite(eyeInfo[e].select, LOW);                        // Chip select

tft->setAddrWindow(XOFFSET[e], YOFFSET[e], 128, 128);
	  
digitalWrite(DISPLAY_DC, HIGH);                      // Data mode

// Now just issue raw 16-bit values for every pixel...
scleraXsave = scleraX; // Save initial X value to reset on each line
irisY       = scleraY - (SCLERA_HEIGHT - IRIS_HEIGHT) / 2;
for(screenY=0; screenY<SCREEN_HEIGHT; screenY++, scleraY++, irisY++) 
	{
	uint16_t *ptr = &dmaBuf[dmaIdx][0];
	scleraX = scleraXsave;
	irisX   = scleraXsave - (SCLERA_WIDTH - IRIS_WIDTH) / 2;
	for(screenX=0; screenX<SCREEN_WIDTH; screenX++, scleraX++, irisX++) 
		{
		uint8_t lidX = screenX;
		if (e == 1)
			lidX = SCREEN_WIDTH-1-lidX;
		if(eyeFreeze || (lower[screenY][lidX] <= lT) || (upper[screenY][lidX] <= uT)) 
			{             // Covered by eyelid
//			p = 0;
			p = righteye[SCREEN_HEIGHT-1-screenY][lidX];
			} 
		else if((irisY < 0) || (irisY >= IRIS_HEIGHT) || (irisX < 0) || (irisX >= IRIS_WIDTH)) 
				{ // In sclera
				p = sclera[scleraY][scleraX];
				} 
			else {                                          // Maybe iris...
				p = polar[irisY][irisX];                        // Polar angle/dist
				d = (iScale * (p & 0x7F)) / 128;                // Distance (Y)
				if(d < IRIS_MAP_HEIGHT) 
					{                       // Within iris area
					a = (IRIS_MAP_WIDTH * (p >> 7)) / 512;        // Angle (X)
					p = iris[d][a];              				 // Pixel = iris
					} 
				else {                                        // Not in iris
					p = sclera[scleraY][scleraX];			 // Pixel = sclera
					}
				}
		*ptr++ = __builtin_bswap16(p); // DMA: store in scanline buffer
		} // end column
	while(dma_busy); // Wait for prior DMA xfer to finish
	descriptor->SRCADDR.reg = (uint32_t)&dmaBuf[dmaIdx] + sizeof dmaBuf[0];
	dma_busy = true;
	dmaIdx   = 1 - dmaIdx;
	dma.startJob();
	} // end scanline

while(dma_busy);  // Wait for last scanline to transmit

digitalWrite(eyeInfo[e].select, HIGH);          // Deselect
SPI.endTransaction();
}


// EYE ANIMATION -----------------------------------------------------------

const uint8_t ease[] = { // Ease in/out curve for eye movements 3*t^2-2*t^3
    0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  3,   // T
    3,  3,  4,  4,  4,  5,  5,  6,  6,  7,  7,  8,  9,  9, 10, 10,   // h
   11, 12, 12, 13, 14, 15, 15, 16, 17, 18, 18, 19, 20, 21, 22, 23,   // x
   24, 25, 26, 27, 27, 28, 29, 30, 31, 33, 34, 35, 36, 37, 38, 39,   // 2
   40, 41, 42, 44, 45, 46, 47, 48, 50, 51, 52, 53, 54, 56, 57, 58,   // A
   60, 61, 62, 63, 65, 66, 67, 69, 70, 72, 73, 74, 76, 77, 78, 80,   // l
   81, 83, 84, 85, 87, 88, 90, 91, 93, 94, 96, 97, 98,100,101,103,   // e
  104,106,107,109,110,112,113,115,116,118,119,121,122,124,125,127,   // c
  128,130,131,133,134,136,137,139,140,142,143,145,146,148,149,151,   // J
  152,154,155,157,158,159,161,162,164,165,167,168,170,171,172,174,   // a
  175,177,178,179,181,182,183,185,186,188,189,190,192,193,194,195,   // c
  197,198,199,201,202,203,204,205,207,208,209,210,211,213,214,215,   // o
  216,217,218,219,220,221,222,224,225,226,227,228,228,229,230,231,   // b
  232,233,234,235,236,237,237,238,239,240,240,241,242,243,243,244,   // s
  245,245,246,246,247,248,248,249,249,250,250,251,251,251,252,252,   // o
  252,253,253,253,254,254,254,254,254,255,255,255,255,255,255,255 }; // n

#ifdef AUTOBLINK
uint32_t timeOfLastBlink = 0L, timeToNextBlink = 0L;
#endif

int FPS_s;
// Process motion for a single frame of left or right eye
void frame(uint16_t iScale) 
{     // Iris scale (0-1023) passed in
static uint32_t frames   = 0; // Used in frame rate calculation
static uint8_t  eyeIndex = 0; // eye[] array counter
int16_t         eyeX, eyeY;
uint32_t        t = micros(); // Time at start of function

if(!(++frames & 255)) 
	{ // Every 256 frames...
	uint32_t elapsed = (millis() - startTime) / 1000;
	int FPS = frames / elapsed;
	if (FPS != FPS_s) 
		{
		Serial.print(FPS); // Print FPS
		Serial.println(" fps");
		FPS_s = FPS;
		}
	}

if(++eyeIndex >= NUM_EYES) 
	eyeIndex = 0; // Cycle through eyes, 1 per call

  // X/Y movement

// Autonomous X/Y eye motion
      // Periodically initiates motion to a new random point, random speed,
      // holds there for random period until next motion.

static boolean  eyeInMotion      = false;
static int16_t  eyeOldX=512, eyeOldY=512, eyeNewX=512, eyeNewY=512;
static uint32_t eyeMoveStartTime = 0L;
static int32_t  eyeMoveDuration  = 0L;

int32_t dt = t - eyeMoveStartTime;      // uS elapsed since last eye event
if(eyeInMotion) 
	{                       // Currently moving?
	if(dt >= eyeMoveDuration) 
		{           // Time up?  Destination reached.
		eyeInMotion      = false;           // Stop moving
		eyeMoveDuration  = random(3000000); // 0-3 sec stop
		eyeMoveStartTime = t;               // Save initial time of stop
		eyeX = eyeOldX = eyeNewX;           // Save position
		eyeY = eyeOldY = eyeNewY;
		} 
	else { // Move time's not yet fully elapsed -- interpolate position
		int16_t e = ease[255 * dt / eyeMoveDuration] + 1;   // Ease curve
		eyeX = eyeOldX + (((eyeNewX - eyeOldX) * e) / 256); // Interp X
		eyeY = eyeOldY + (((eyeNewY - eyeOldY) * e) / 256); // and Y
		}
	}
else {                                // Eye stopped
	eyeX = eyeOldX;
	eyeY = eyeOldY;
	if(dt > eyeMoveDuration) 
		{            // Time up?  Begin new move.
		int16_t  dx, dy;
		uint32_t d;
		do 	{                                // Pick new dest in circle
			eyeNewX = random(1024);
			eyeNewY = random(1024);
			dx      = (eyeNewX * 2) - 1023;
			dy      = (eyeNewY * 2) - 1023;
			} while (((d = (dx * dx + dy * dy)) > (1023 * 1023)) && (eyeNewX < 512)); // Keep trying
		eyeMoveDuration  = random(72000, 144000); // ~1/14 - ~1/7 sec
		eyeMoveStartTime = t;               // Save initial time of move
		eyeInMotion      = true;            // Start move on next frame
		}
	}


  // Blinking

#ifdef AUTOBLINK
// Similar to the autonomous eye movement above -- blink start times
// and durations are random (within ranges).
if((t - timeOfLastBlink) >= timeToNextBlink) 
	{ // Start new blink?
	timeOfLastBlink = t;
	uint32_t blinkDuration = random(36000, 72000); // ~1/28 - ~1/14 sec
	// Set up durations for both eyes (if not already winking)
	for(uint8_t e=0; e<NUM_EYES; e++) 
		{
		if(eye[e].blink.state == NOBLINK) 
			{
			eye[e].blink.state     = ENBLINK;
			eye[e].blink.startTime = t;
			eye[e].blink.duration  = blinkDuration;
			}
		}
	timeToNextBlink = blinkDuration * 3 + random(4000000);
	}
#endif

if(eye[eyeIndex].blink.state) 
	{ // Eye currently blinking?
	// Check if current blink state time has elapsed
	if((t - eye[eyeIndex].blink.startTime) >= eye[eyeIndex].blink.duration) 
		{
		// Yes -- increment blink state, unless...
		if((eye[eyeIndex].blink.state == ENBLINK) && ( // Enblinking and...
		((eyeInfo[eyeIndex].wink >= 0) &&
		digitalRead(eyeInfo[eyeIndex].wink) == LOW) )) 
			{
			// Don't advance state yet -- eye is held closed instead
			} 
		else { // No buttons, or other state...
			if(++eye[eyeIndex].blink.state > DEBLINK) 
				{ // Deblinking finished?
				eye[eyeIndex].blink.state = NOBLINK;      // No longer blinking
				} 
			else { // Advancing from ENBLINK to DEBLINK mode
				eye[eyeIndex].blink.duration *= 2; // DEBLINK is 1/2 ENBLINK speed
				eye[eyeIndex].blink.startTime = t;
				}
			}
		}
	} 
else { // Not currently blinking...check buttons!

	if((eyeInfo[eyeIndex].wink >= 0) && (digitalRead(eyeInfo[eyeIndex].wink) == LOW)) 
		{ // Wink!
		eye[eyeIndex].blink.state     = ENBLINK;
		eye[eyeIndex].blink.startTime = t;
		eye[eyeIndex].blink.duration  = random(45000, 90000);
		}
	}

// Process motion, blinking and iris scale into renderable values

// Iris scaling: remap from 0-1023 input to iris map height pixel units
iScale = ((IRIS_MAP_HEIGHT + 1) * 1024) /
	 (1024 - (iScale * (IRIS_MAP_HEIGHT - 1) / IRIS_MAP_HEIGHT));

// Scale eye X/Y positions (0-1023) to pixel units used by drawEye()
eyeX = map(eyeX, 0, 1023, 0, SCLERA_WIDTH  - 128);
eyeY = map(eyeY, 0, 1023, 0, SCLERA_HEIGHT - 128);
//if (eyeIndex == 1) 
//	eyeX = (SCLERA_WIDTH - 128) - eyeX; // Mirrored display

// Horizontal position is offset so that eyes are very slightly crossed
// to appear fixated (converged) at a conversational distance.  Number
// here was extracted from my posterior and not mathematically based.
// I suppose one could get all clever with a range sensor, but for now...
if (NUM_EYES > 1) 
	eyeX += 4;
if (eyeX > (SCLERA_WIDTH - 128)) 
	eyeX = (SCLERA_WIDTH - 128);

// Eyelids are rendered using a brightness threshold image.  This same
// map can be used to simplify another problem: making the upper eyelid
// track the pupil (eyes tend to open only as much as needed -- e.g. look
// down and the upper eyelid drops).  Just sample a point in the upper
// lid map slightly above the pupil to determine the rendering threshold.
static uint8_t uThreshold = 128;
uint8_t        lThreshold, n;
#ifdef TRACKING
int16_t sampleX = SCLERA_WIDTH  / 2 - (eyeX / 2), // Reduce X influence
	sampleY = SCLERA_HEIGHT / 2 - (eyeY + IRIS_HEIGHT / 4);
// Eyelid is slightly asymmetrical, so two readings are taken, averaged
if (sampleY < 0) 
	n = 0;
else n = (upper[sampleY][sampleX] + upper[sampleY][SCREEN_WIDTH - 1 - sampleX]) / 2;
//else n = (*(upperp+sampleY*screen_width+sampleX) + *(upperp+sampleY*screen_width+screen_width - 1 - sampleX)) / 2;
//else n=0;
uThreshold = (uThreshold * 3 + n) / 4; // Filter/soften motion
// Lower eyelid doesn't track the same way, but seems to be pulled upward
// by tension from the upper lid.
lThreshold = 254 - uThreshold;
#else // No tracking -- eyelids full open unless blink modifies them
uThreshold = lThreshold = 0;
#endif

// The upper/lower thresholds are then scaled relative to the current
// blink position so that blinks work together with pupil tracking.
if(eye[eyeIndex].blink.state) 
	{ // Eye currently blinking?
	uint32_t s = (t - eye[eyeIndex].blink.startTime);
	if(s >= eye[eyeIndex].blink.duration) s = 255;   // At or past blink end
	else s = 255 * s / eye[eyeIndex].blink.duration; // Mid-blink
	s          = (eye[eyeIndex].blink.state == DEBLINK) ? 1 + s : 256 - s;
	n          = (uThreshold * s + 254 * (257 - s)) / 256;
	lThreshold = (lThreshold * s + 254 * (257 - s)) / 256;
	} 
else {
	n          = uThreshold;
	}

// Pass all the derived values to the eye-rendering function:
drawEye(eyeIndex, iScale, eyeX, eyeY, n, lThreshold);
}


// AUTONOMOUS IRIS SCALING (if no photocell or dial) -----------------------

#if !defined(LIGHT_PIN) || (LIGHT_PIN < 0)

// Autonomous iris motion uses a fractal behavior to similate both the major
// reaction of the eye plus the continuous smaller adjustments that occur.

uint16_t oldIris = (IRIS_MIN + IRIS_MAX) / 2, newIris;

void split( // Subdivides motion path into two sub-paths w/randomization
  int16_t  startValue, // Iris scale value (IRIS_MIN to IRIS_MAX) at start
  int16_t  endValue,   // Iris scale value at end
  uint32_t startTime,  // micros() at start
  int32_t  duration,   // Start-to-end time, in microseconds
  int16_t  range) 
  
{    // Allowable scale value variance when subdividing

if(range >= 8) // Limit subdivision count, because recursion
	{     
	range    /= 2;     // Split range & time in half for subdivision,
	duration /= 2;     // then pick random center point within range:
	int16_t  midValue = (startValue + endValue - range) / 2 + random(range);
	uint32_t midTime  = startTime + duration;
	split(startValue, midValue, startTime, duration, range); // First half
	split(midValue  , endValue, midTime  , duration, range); // Second half
	} 
else {             // No more subdivisions, do iris motion...
	int32_t dt;        // Time (micros) since start of motion
	int16_t v;         // Interim value
	while((dt = (micros() - startTime)) < duration) 
		{
		v = startValue + (((endValue - startValue) * dt) / duration);
		if(v < IRIS_MIN)      
			v = IRIS_MIN; // clip just in case
		else if(v > IRIS_MAX) 
			v = IRIS_MAX;
		frame(v);        // Draw frame w/interim iris scale value
		}
	}
}

#endif // !LIGHT_PIN


// MAIN LOOP -- runs continuously after setup() ----------------------------

void loop() {
char *p;
if (Serial.available() && (p=readline()))
     {
	Serial.println();
     char *p1;
     p1 = strtok (p," ");
     if (!strcmp(p1, "?"))
          menu();
     else if (!strcmp(p1, "EF"))
          {
          eyeFreeze = !eyeFreeze;
		Serial.print("eyeFreeze ");
		Serial.println(eyeFreeze);
          }
     else if (!strcmp(p1, "M0"))
          {
          p = strtok (NULL, " ");
          char * pEnd;
          int t = strtol(p,&pEnd,0);
		MADCTL[0] = t;
          Serial.print("Set MADCTL 0=0x");
		Serial.println(t,HEX);
          }
     else if (!strcmp(p1, "M1"))
          {
          p = strtok (NULL, " ");
          char * pEnd;
          int t = strtol(p,&pEnd,0);
		MADCTL[1] = t;
          Serial.print("Set MADCTL 1=0x");
		Serial.println(t,HEX);
          }
     else if (!strcmp(p1, "X0"))
          {
          p = strtok (NULL, " ");
          char * pEnd;
          int t = strtol(p,&pEnd,0);
		if (t)
			{
			drawFace();
			XOFFSET[0] = t;
			Serial.print("Set ");
			}
		else t = XOFFSET[0];
          Serial.print("XOFFSET 0= ");
		Serial.println(t);
          }
     else if (!strcmp(p1, "X1"))
          {
          p = strtok (NULL, " ");
          char * pEnd;
          int t = strtol(p,&pEnd,0);
		if (t)
			{
			drawFace();
			XOFFSET[1] = t;
			Serial.print("Set ");
			}
		else t = XOFFSET[1];
          Serial.print("XOFFSET 1= ");
		Serial.println(t);
          }
     else if (!strcmp(p1, "Y0"))
          {
          p = strtok (NULL, " ");
          char * pEnd;
          int t = strtol(p,&pEnd,0);
		if (t)
			{
			drawFace();
			YOFFSET[0] = t;
			Serial.print("Set ");
			}
		else t = YOFFSET[0];
          Serial.print("YOFFSET 0= ");
		Serial.println(t);
          }
     else if (!strcmp(p1, "Y1"))
          {
          p = strtok (NULL, " ");
          char * pEnd;
          int t = strtol(p,&pEnd,0);
		if (t)
			{
			drawFace();
			YOFFSET[1] = t;
			Serial.print("Set ");
			}
		else t = YOFFSET[1];
          Serial.print("YOFFSET 1= ");
		Serial.println(t);
          }
	else {
		Serial.print("unrecognized command ");
		Serial.println(p);
		}
	}
 
#if defined(LIGHT_PIN) && (LIGHT_PIN >= 0) // Interactive iris

  int16_t v = analogRead(LIGHT_PIN);       // Raw dial/photocell reading
#ifdef LIGHT_PIN_FLIP
  v = 1023 - v;                            // Reverse reading from sensor
#endif
  if(v < LIGHT_MIN)      v = LIGHT_MIN;  // Clamp light sensor range
  else if(v > LIGHT_MAX) v = LIGHT_MAX;
  v -= LIGHT_MIN;  // 0 to (LIGHT_MAX - LIGHT_MIN)
#ifdef LIGHT_CURVE  // Apply gamma curve to sensor input?
  v = (int16_t)(pow((double)v / (double)(LIGHT_MAX - LIGHT_MIN),
    LIGHT_CURVE) * (double)(LIGHT_MAX - LIGHT_MIN));
#endif
  // And scale to iris range (IRIS_MAX is size at LIGHT_MIN)
  v = map(v, 0, (LIGHT_MAX - LIGHT_MIN), IRIS_MAX, IRIS_MIN);
#ifdef IRIS_SMOOTH // Filter input (gradual motion)
  static int16_t irisValue = (IRIS_MIN + IRIS_MAX) / 2;
  irisValue = ((irisValue * 15) + v) / 16;
  frame(irisValue);
#else // Unfiltered (immediate motion)
  frame(v);
#endif // IRIS_SMOOTH

#else  // Autonomous iris scaling -- invoke recursive function

  newIris = random(IRIS_MIN, IRIS_MAX);
  split(oldIris, newIris, micros(), 10000000L, IRIS_MAX - IRIS_MIN);
  oldIris = newIris;

#endif // LIGHT_PIN

}

#define INPBUFSIZE  150
char buffer[INPBUFSIZE+1];

char *readline(void)
{
if (readline(buffer, INPBUFSIZE, 50) > 0)
	return buffer;
else return 0;
//return readline(buffer, INPBUFSIZE, 50);
}

uint16_t readline(char * buf, uint16_t bufsize, uint16_t timeout)
{
uint16_t replyidx = 0;
while (timeout--) 
     {
     while(Serial.available()) 
          {
          char c = Serial.read();
          //SerialDebug.println(c);
          if (c == '\r') 
               continue;
          if (c == '\n') 
               {
               // the first '\n' is ignored
//               if (replyidx == 0) 
//                    continue;
               timeout = 0;
               break;
               }
          buf[replyidx] = c;
          replyidx++;
          
          // Buffer is full
          if (replyidx >= bufsize) 
               {
               Serial.println("*overflow*");   // for my debuggin' only!
               timeout = 0;
               break;
               }
          }

     // delay if needed
     if (timeout) 
          delay(1);
     }

buf[replyidx] = 0;  // null term
return replyidx;
}
#define BUFFPIXEL 20

void bmpDraw(char *filename, uint8_t x, uint16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  if((x >= tft->width()) || (y >= tft->height())) return;

  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');

  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found"));
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft->width())  w = tft->width()  - x;
        if((y+h-1) >= tft->height()) h = tft->height() - y;

        // Set TFT address window to clipped image bounds
        tft->startWrite(); // Start TFT transaction
        tft->setAddrWindow(x, y, w, h);

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            tft->endWrite(); // End TFT transaction
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
            tft->startWrite(); // Start new TFT transaction
          }

          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              tft->endWrite(); // End TFT transaction
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
              tft->startWrite(); // Start new TFT transaction
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft->pushColor(tft->color565(r,g,b));
          } // end pixel
        } // end scanline
        tft->endWrite(); // End last TFT transaction
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }

  bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}
void menu()
{
Serial.println("        Menu");
Serial.println("============================================");
Serial.println("?      Show this Menu");
Serial.println("default set eye to default");
Serial.println("M0 n   set MATCTL 0 to n");
Serial.println("M1 n   set MATCTL 1 to n");
Serial.println("X0 n   set XOFFSET 0 to n");
Serial.println("X1 n   set XOFFSET 1 to n");
Serial.println("Y0 n   set YOFFSET 0 to n");
Serial.println("Y1 n   set YOFFSET 1 to n");
Serial.println("EF     toggle eyeFreeze");
Serial.println();
Serial.print("MADCTL 0,1 0x");
Serial.print(MADCTL[0],HEX);
Serial.print(", 0x");
Serial.println(MADCTL[1],HEX);

}
