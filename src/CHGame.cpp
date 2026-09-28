#include "CHGame.h"

CHGame::CHGame() : CHGfx() {}


uint8_t currentButtonState = 0;
uint8_t previousButtonState = 0;

uint8_t CHGame::buttonsState() {

	// FIX: was `uint8_t buttons;` - uninitialised, so any bit not set below
	// kept whatever was on the stack and phantom buttons came and went.
	uint8_t buttons = 0;
	if (digitalRead(PIN_BTN_A) == LOW) {
		buttons |= A_BUTTON;
	}
	if (digitalRead(PIN_BTN_B) == LOW) {
		buttons |= B_BUTTON;
	}
	if (digitalRead(PIN_BTN_LEFT) == LOW) {
		buttons |= LEFT_BUTTON;
	}
	if (digitalRead(PIN_BTN_RIGHT) == LOW) {
		buttons |= RIGHT_BUTTON;
	}
	if (digitalRead(PIN_BTN_UP) == LOW) {
		buttons |= UP_BUTTON;
	}
	if (digitalRead(PIN_BTN_DOWN) == LOW) {
		buttons |= DOWN_BUTTON;
	}
	if (digitalRead(PIN_BTN_SELECT) == LOW) {
		buttons |= SELECT_BUTTON;
	}
	if (digitalRead(PIN_BTN_START) == LOW) {
		buttons |= START_BUTTON;
	}
	return buttons;
}

void CHGame::boot() {

	pinMode(PIN_BTN_A, INPUT_PULLUP);
	pinMode(PIN_BTN_B, INPUT_PULLUP);
	pinMode(PIN_BTN_UP, INPUT_PULLUP);
	pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
	pinMode(PIN_BTN_LEFT, INPUT_PULLUP);
	pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);
	pinMode(PIN_BTN_START, INPUT_PULLUP);
	pinMode(PIN_BTN_SELECT, INPUT_PULLUP);

	currentButtonState = buttonsState();
	previousButtonState = currentButtonState;

}

bool CHGame::anyPressed(uint8_t buttons) {
	return (buttonsState() & buttons) != 0;
}

bool CHGame::pressed(uint8_t buttons) {
	return (buttonsState() & buttons) == buttons;
}

bool CHGame::notPressed(uint8_t buttons) {
	return (buttonsState() & buttons) == 0;
}

bool CHGame::justPressed(uint8_t buttons) {
  	return (!(previousButtonState & buttons) && (currentButtonState & buttons));
}

bool CHGame::justReleased(uint8_t buttons) {
	// FIX: was !(previous) && !(current), i.e. "not held on either frame".
	// Released means held last frame and not held now.
	return ((previousButtonState & buttons) && !(currentButtonState & buttons));
}

void CHGame::pollButtons() {

	previousButtonState = currentButtonState;
	currentButtonState = buttonsState();

}

void CHGame::setFrameRate(uint8_t frameRate) {

	this->targetFPS = frameRate;
	this->frameDelay = 1000 / this->targetFPS;
	this->lastFrameTime = millis();

}

bool CHGame::nextFrame() {

	uint32_t currentMillis = millis();

	if (currentMillis - this->lastFrameTime >= this->frameDelay) {
		this->lastFrameTime = currentMillis;
		this->frameCount++;
		return true;
	} 
	else {
		return false;
	}

}

void CHGame::resetFrameCount() { this->frameCount = 0; }

uint16_t CHGame::getFrameCount() const { 
	return this->frameCount; 
}

uint16_t CHGame::getFrameCount(uint16_t mod) const {
	return this->frameCount % mod;
}

bool CHGame::getFrameCountHalf(uint8_t mod) const {
	return getFrameCount(mod) > (mod / 2);
}

bool CHGame::isFrameCount(uint16_t mod) const {
	return (this->frameCount % mod) == 0;
}

bool CHGame::isFrameCount(uint16_t mod, uint16_t val) const {
	return (this->frameCount % mod) == val;
}

/* ----------------------------------------------------------------------------
 *  Draw a horizontal dotted line.
 */
void CHGame::drawHorizontalDottedLine(uint8_t x1, uint8_t x2, uint8_t y,
                                      uint8_t colour) {
	uint8_t diff = (x2 - x1);

	for (uint8_t x = 0; x <= diff; x += 2) {
		drawPixel(x1 + x, y, colour);
	}

}

/* ----------------------------------------------------------------------------
 *  Draw a vertical dotted line.
 */
void CHGame::drawVerticalDottedLine(uint8_t y1, uint8_t y2, uint8_t x,
                                    uint8_t colour) {
	uint8_t diff = (y2 - y1);

	for (uint8_t y = 0; y <= diff; y += 2) {
		drawPixel(x, y1 + y, colour);
	}

}
