#ifndef GLOBAL_DEF_H_INCLUDED
#define GLOBAL_DEF_H_INCLUDED

//!!! Global definitions (no variables initialized here)

//#define SSD1306_32 // Add at compile time if screen size is 128x32

// Size of the array to store decimal/string (length + '\0' + length_storage)
#define DIS_STR (5 + 1 + 1)
// Total number of the digits in the string (length_storage/holder position)
#define DIS_STR_NOD (DIS_STR - 1)

// Fonts and images sets.
// Represents indexes for the table of pointers (fonts_set_pp from fonts_symbols.h),
// from where begins descriptions of the fonts.
#define FONT_8PX 0
#define IMGS_8PX (FONT_8PX + 3)
#define SIGNS_8PX (IMGS_8PX + 3)
#define IMGS_24PX (SIGNS_8PX + 3)
#define FONT_32PX (IMGS_24PX + 3)

#define FONT_ALIGN_RIGHT_BIT 6
#define FONT_NO_DRAW_BIT 7
#define ALIGN_RIGHT (1 << FONT_ALIGN_RIGHT_BIT)
#define ALIGN_LEFT  (0 << FONT_ALIGN_RIGHT_BIT)
#define NO_DRAW (1 << FONT_NO_DRAW_BIT)

#endif // GLOBAL_DEF_H_INCLUDED
