#pragma once

#include <CHGfx.h> 

#ifndef A_BUTTON
	#define A_BUTTON 1u << 0
	#define B_BUTTON 1u << 1
	#define UP_BUTTON 1u << 2
	#define DOWN_BUTTON 1u << 3
	#define LEFT_BUTTON 1u << 4
	#define RIGHT_BUTTON 1u << 5
	#define START_BUTTON 1u << 6
	#define SELECT_BUTTON 1u << 7
#endif

class CHGame : public CHGfx  {

	private:

		uint8_t targetFPS = 30; 
		uint16_t frameDelay = 1000 / targetFPS;
		// CHANGE: was uint64_t. millis() is 32-bit and wraps cleanly under
		// unsigned subtraction; 64-bit maths is a library call on RV32.
		uint32_t lastFrameTime = 0;

	public:

		uint32_t frameCount = 0;

		CHGame();


		uint8_t buttonsState();
		void boot();

		bool anyPressed(uint8_t buttons);
		bool pressed(uint8_t buttons);
		bool notPressed(uint8_t buttons);
		bool justPressed(uint8_t buttons);
		bool justReleased(uint8_t buttons);
		void pollButtons();

		void setFrameRate(uint8_t frameRate);

		bool nextFrame();
		uint16_t getFrameCount() const;
		uint16_t getFrameCount(uint16_t mod) const;
		bool getFrameCountHalf(uint8_t mod) const;
		bool isFrameCount(uint16_t mod) const;
		bool isFrameCount(uint16_t mod, uint16_t val) const;

		void clearButtonState();
		void resetFrameCount();

		void drawHorizontalDottedLine(uint8_t x1, uint8_t x2, uint8_t y, uint8_t colour);
		void drawVerticalDottedLine(uint8_t y1, uint8_t y2, uint8_t x, uint8_t colour);

};