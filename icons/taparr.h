#ifndef TAPARR_H
#define TAPARR_H

#include <stdint.h>

// Define GIMP types
typedef unsigned int guint;
typedef uint8_t guint8;

// Define the structure type
struct TapImage {
  guint width;
  guint height;
  guint bytes_per_pixel;
  guint8 pixel_data[150 * 150 * 4 + 1];
};

// Declare the tap bitmap from taparr.c
extern const struct TapImage taparr;

#endif // TAPARR_H
