#pragma once


namespace PS4_BUTTONS {

static const int CROSS    = 0;
static const int CIRCLE   = 1;
static const int TRIANGLE = 2;
static const int SQUARE   = 3;
static const int L1       = 4;
static const int R1       = 5;
static const int L2       = 6;
static const int R2       = 7;
static const int SHARE    = 8;
static const int OPTIONS  = 9;
static const int PS       = 10;
static const int L3       = 11;   // left stick
static const int R3       = 12;   // right stick

}   // namespace PS4_BUTTONS

namespace PS4_AXES {

static const int LEFT_WEST   = 0;
static const int LEFT_NORTH  = 1;
static const int L2          = 2;   // 1.0 when released, -1.0 when fully pressed
static const int RIGHT_WEST  = 3;
static const int RIGHT_NORTH = 4;
static const int R2          = 5;   // 1.0 when released, -1.0 when fully pressed
static const int DPAD_WEST   = 6;   // 1.0 when left, -1.0 when right, else 0.0
static const int DPAD_NORTH  = 7;   // 1.0 when up, -1.0 when down, else 0.0

}   // namespace PS4_AXES