# Oxy_p
Pulse oximeter
is a DIY pulse oximeter that indirectly (by LED-powered sensors) monitors
oxygen saturation of a patient's blood and monitors heart rate.

![Oxy_p](drawings/oxy_p_logo.svg)

## About device
Oxy_p has OLED display 0.96" (24 mm), 2 buttons, battery pack (3xAA) and
external wired (0.8 m) optical sensor.

**Characteristics**
- Modes: Heart Rate (HR) | SpO2 + HR | Off
- Max display heart rate: 255 beats/min
- Max display SpO2: 99%
- Power: max 5V DC, min 3V DC
- Typical power consumption (HR mode): about 11 mA at 4.5V
- Power consumption in power-down: <0.5 uA at 4.5V
- Max single LED current: 51 mA (255 steps in 0.2 mA)
- Scale of the measurements: 2048 | 4096 | 8192 | 16384 nA
- Sample rate: 50 samples/sec
- UART output: 38400 Bd, 8 bits data, 2 stop bits, async, no parity
- Sensor: MAX30102 (I2C address 0xAF+0xAE)
- Screen driver: SSD1306, 128x64 (I2C address 0x7B+0x7A)

## Usage
Power up device or hold Button 1 down more than 3 sec. The display should blink
once (about 0.5 sec) and big "Heart" symbol will appear on the right side of
the screen.
Place finger on the sensor's window and wrap fixing strap around the finger.
Do not move the finger and sensor. Wait until stable photoplethysmogram 
(about 4 sec long) will be drawn in the left corner of the screen.
Calculation of the heart rate will take about 15-20 sec. For more precise
result wait for 1 minute. Averaged heart rate will be shown below the big
"Heart" symbol, in beats per minute.
Hold Button 1 more than 3 sec to power off the device. Blank display indicates
that device was powered off.

 * To switch device into "SpO2 + HR" mode: hold Button 2 more than 1 sec. Big
"Heart" becomes filled.
 * To switch device into "Heart rate" mode: hold Button 1 more than 1 sec. Big
"Heart" becomes empty.
 * To increase current of the sensor's LEDs ("brightness") press Button 2.
 * To decrease current of the sensor's LEDs ("brightness") press Button 1.
 * To switch scale of the measurements: hold Button 2 more than 1 sec while in
"SpO2 + HR" mode.

The scale indicator located on top of the screen and has next meaning:
 + no_heart symbol - 2048 nA;
 + empty_heart - 4096 nA;
 + half-filled - 8192 nA;
 + full-filled - 16384 nA.

## Firmware
Release: [oxy_p/releases](https://github.com/SuslikV/oxy_p/releases)

Original firmware was build with

IDE: CodeBlocks 20.03-r11983

SDK: avr-gcc-10.1.0

## Algorithm
The main program written in C for ATmega AVR (MCU ATmega88PA).

### TWI bus usage logic
TWI (I2C compatible) bus is used for communication between modules and MCU.
MCU running in Master mode for TWI bus, sensor and display are Slave devices.
TWI transfers are interrupt driven. The implementation is based on heavily
modified AN2480 (AVR315: Using the TWI Module as I2C Master) document. Main
difference from the base code is the ability to handle REPEATED START signal
and send STOP with forced timeout.

Driver utilizes hardware block (TWI) of the MCU for the bus transfers. Driver
of the TWI bus allocates buffer of 130 bytes for internal use. MCU prepares
data that needs to be sent. Driver buffers this data and starts asynchronous
transfer. If new transfer requested from MCU, driver waits until previous
transfer will complete. If transfer stalled - driver resets the TWI hardware
block of the MCU. Important messages re-sends automatically in case of bus
failures.

#### Big-endian vs Little-endian
Data handling in transfers from the MAX30102 sensor to ATmega MCU.

The data buffer of the sensor contains left-justified 18-bit samples in 24-bit
space (3 bytes). Each sample chosen to have maximum resolution of 16-bit just
to simplify math calculations by the 8-bit MCU.

Single sample composition in the sensor's buffer (16-bit resolution):
```
byte1 = [  -,  -,  -,  -,  -,  -, 15, 14]
byte2 = [ 13, 12, 11, 10,  9,  8,  7,  6]
byte3 = [  5,  4,  3,  2,  1,  0,  -,  -]
```

Here MCU buffer represents byte data obtained from the sensor that is running
in the HR mode.
```
    array[x, b1, b2, b3, b1, b2, b3, ...]
             |sample1 |  |sample2 |
             |  LED1  |  |  LED1  |

x - I2C address
b1 - byte1
b2 - byte2
b3 - byte3
```

-----------------------------------------
STEP 1 (get LOW/HIGH)
```
array[3] >>= 2;
array[3] |= array[2] << 6; // low byte

array[2] >>= 2;
array[2] |= array[1] << 6; // high byte
```

-----------------------------------------
STEP 2 (copy LOW)
```
    array[x, b1, high1, low1, b1, high2, low2, ...]
             |-- sample1 --|  |-- sample2 --|
             |     LED1    |  |     LED1    |
```
			 
The sensor returns Big-endian (the first byte is the highest).
ATmega MCU, in memory, expects: low byte then high byte.
Thus, make "b1 = low" to cast to "unsigned short int" without byte swap.
```
array[1] = array[3];
```

-----------------------------------------
RESULT
```
    array[x, low1, high1, low1, low2, high2, low2, ...]
             | sample1 |        | sample2 |
             |   LED1  |        |   LED1  |
```

Thus, the array[1] -> points to the sample N1 of the type "unsigned short int".

#### Display driver logic
ATmega88PA driver for the screen based on SSD1306.

The display driver is capable to handle proportional fonts.
The display driver works only with 1-bit depth bitmaps.
The height of the bitmap images is a multiple of 8 (page size).

The data that represents each font are stored in byte arrays of:
- images(pictures of the glyhps);
- pointers to the start of each image;
- width and symmetry;
- height and spacing properties for sets of images.

All these arrays organized into a single array of pointers that represents all
available images that intended for display.

To reduce storage space for the glyphs, each image has horizontal symmetry
property and can be stored half-sized.

To be able to show a string of symbols on the screen the display driver
requires next info:
- the font's number (selected font);
- pointer to the array where each position represents ordered number
of the glyph from the font set. Note: special fixed position (DIS_STR_NOD)
of the array stores total number of the elements in the string;
- start/end column and start page of the area intended for display;
- horizontal alignment of the displayed string in the displayed area (effect).

During the string display next steps performed:
1. Resolves the font and properties pointers.
2. Calculates length of the sting in pixels.
3. Blanks display area left to the string (depends on string alignment).
4. The string splits into characters. Each character draws separately.
   - 4.1. Calculates stored data length of the each character according to its
   symmetry.
   - 4.2. Displays spacing before the current character (TWI).
   - 4.3. Fills the buffer with stored data of the glyph image (half of the
   image or full image).
   - 4.4. Check for symmetry and parity of the glyph final width.
      - 4.4.1. If the glyph has symmetry and has odd number of columns - copy
      each column starting from the "last - 1" to the first one (mirroring).
      - 4.4.1. If the glyph has symmetry and has even number of columns - copy
      each column starting from the "last" to the first one (mirroring).
    - 4.5. Display final contents of the buffer (TWI).
5. Blanks display area right to the string.
6. Returns length of the string in pixels.

### Heart rate and SpO2 calculation logic
Samples filtered using exponential filter.
#### Beats detector
Heart beats detector algorithm is looking for extremums on rise/fall edge of
the flow of samples. Front of the signal filtered more than back. Transition
from max to min means beat detected (systole).

#### Heart rate
Heart rate calculates continuously. In 15 sec intervals (each quarter minute)
heart rate measurements are stored in cycled buffer. Final value of heart rate
is sum of available quarters multiplied by weight of the quarter:
15 sec -> 4/1;
30 sec -> 4/2;
45 sec -> 4/3;
60 sec -> 4/4.
Heart rate monitor renders normal photoplethysmogram about 4 sec long (blood
rich areas highlighted).

#### SpO2
SpO2 calculated via getting 1/R ratio of light reflected from the skin (finger)
for red and infrared LEDs of the sensor. Then 1/R linearly approximates to
actual SpO2 percentage. R = (AC_red / DC_red) / (AC_ir / DC_ir).

### Buttons logic
To acquire buttons state - timer started on each pin interrupt.
Each pin interrupt prolongs the timer's countdown.
The timer counts the "debouncing" interval to ensure that the triggered pin's
level stabilized.
At the end of the countdown the state of each pin analyzed and remembered.
When debouncing period ends and all pins remains released - the debouncing
timer halts.

### UART communication logic
Device outputs diagnostic and error messages via UART as well as unfiltered
data samples. Main flow of the output is raw LED sample data in hex format
(16-bit, 4 symbols). First number is LED1 (usually IR), second number is LED2
(usually RED) data. One LED per line. If LED2 not used (HR mode) then only one
number will be sent via UART.

Errors list:

`!Ei` - error getting info/id from the sensor

`!Ep` - error getting data pointers from the sensor

`!Ed` - error getting sample data from the sensor

## Parts, Circuit, Board
Original DIY board and circuits from the `boards_and_circuits` dir,
were designed using

EDA: KiCad v5.1.9

Vector graphics editor: Inkscape v0.92

**Parts:**
- Sensor: module MAX30102 (includes MAX30102 IC, stabilizers, filters etc) x1
- Display: module SSD1306 128x64, 22x11mm (includes SSD1306 controller + OLED 
matrix, stabilizers, filters etc) x1
- IC: ATmega88PA (TQFP) x1
- Battery holder: 3x1.5 AA x1
- Cable/wires: patch-cord 0.8m, 0.22mm2, stranded
- Board: DIY PCB 50x25x1mm (see `oxy_p_main_board` dir in archives) x1
- Sensor communication board: DIY PCB 35x15x1mm (see `pulse_sensor_board` dir
in archives) x1
- Programmer: DIY programmer for ATmega AVR (see avr_prog_gs in archives) x1
- Capacitors: 0805 0.1u x2
- Resistors: 0805 10k x3, 47k x1, 100k x1; 1206 0R x2
- Transistors: SOT-23 p-n-p BC857C x1; SO-8 n-ch(2) IRF7103 x1
- Diode: SS24 (1N5822) x1
- Inductor: 10uH (100mA max) x1
- Buttons: TL3342 x2

# The name meaning
"Oxy_p" stands for "**OXY**gen saturation and **P**ulse monitor".
Fist letter is capital (in most cases). Pronounce "`äksə-pi".

# Why?
It was designed to be part of some trashy DIY YouTube video.
Originally, only heart rate was in the scope of interest (with self-made
sensor). But I was late. Feb came... So, project was frozen - all DIY dreams
disappeared. But TWI driver was written, programmer was made, screen driver
complete.

# License
As is. It is free.
