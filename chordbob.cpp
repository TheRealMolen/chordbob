#include "daisy_seed.h"
#include "daisysp.h"
#include "Effects/reverbsc.h"

using namespace daisy;
using namespace daisysp;

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;

constexpr float SampleRate = 48000;
constexpr int SamplesPerBlock = 4;

DaisySeed hw;

I2CHandle key_i2c_handle;
SpiHandle rgb_spi_handle;
GPIO rgb_spi_cs;
struct RgbSpiBuf
{
	static constexpr uint32_t Width = 4;
	static constexpr uint32_t Height = 4;
	static constexpr uint32_t NumLeds = Width * Height;
	static constexpr uint32_t RgbiMask = 0x000000e0;	// all ibgr values must be ORd with this
	static constexpr uint8_t BaseIntensity = 3;
	uint32_t startFrame = 0;

	uint32_t rgbi[NumLeds];

	uint32_t endFrame = ~0u;
};
RgbSpiBuf rgb_spi_buf;


void updateKeypadBlocking();

void initKeypad()
{
	// init our keyboard input over I2C
	I2CHandle::Config keyCfg;
	keyCfg.periph = I2CHandle::Config::Peripheral::I2C_1;
	keyCfg.speed  = I2CHandle::Config::Speed::I2C_400KHZ;
	keyCfg.mode   = I2CHandle::Config::Mode::I2C_MASTER;
	keyCfg.pin_config.scl  = seed::D11;
	keyCfg.pin_config.sda  = seed::D12;
	key_i2c_handle.Init(keyCfg);

	// init our RGB buffer
	for (uint32_t i=0; i < RgbSpiBuf::NumLeds; ++i)
		rgb_spi_buf.rgbi[i] = RgbSpiBuf::RgbiMask | RgbSpiBuf::BaseIntensity;

	// init our SPI setup so we can set RGB values
	SpiHandle::Config rgbCfg;

	rgbCfg.mode = SpiHandle::Config::Mode::MASTER;
	rgbCfg.periph = SpiHandle::Config::Peripheral::SPI_1;

	rgbCfg.pin_config.sclk = seed::D8;
	rgbCfg.pin_config.miso = Pin();	// unused
	rgbCfg.pin_config.mosi = seed::D10;
	rgbCfg.pin_config.nss = Pin();

	rgbCfg.direction = SpiHandle::Config::Direction::TWO_LINES_TX_ONLY;	// it's a write-only device

	rgb_spi_handle.Init(rgbCfg);

	rgb_spi_cs.Init(seed::D7, GPIO::Mode::OUTPUT);
	rgb_spi_cs.Write(true);

	updateKeypadBlocking();
}

void setKeypadPixelIntensity(uint32_t x, uint32_t y, uint32_t i)
{
	if (x >= RgbSpiBuf::Width || y >= RgbSpiBuf::Height)
		return;
	if (i >= 31)
		i = 31;

	uint32_t ix = x + (y * RgbSpiBuf::Width);
	uint32_t oldRgbx = rgb_spi_buf.rgbi[ix] & (RgbSpiBuf::RgbiMask | 0xffffff00);
	rgb_spi_buf.rgbi[ix] = oldRgbx | i;
}

void setKeypadPixel(uint32_t x, uint32_t y, uint8_t r, uint8_t g, uint8_t b, uint8_t i = RgbSpiBuf::BaseIntensity)
{
	if (x >= RgbSpiBuf::Width || y >= RgbSpiBuf::Height)
		return;
	if (i >= 31)
		i = 31;
	
	uint32_t ix = x + (y * RgbSpiBuf::Width);
	const uint32_t rgbi = RgbSpiBuf::RgbiMask | (uint32_t(r) << 24) | (uint32_t(g) << 16) | (uint32_t(b) << 8) | i;
	rgb_spi_buf.rgbi[ix] = rgbi;
}


void updateKeypadBlocking()
{
	rgb_spi_cs.Write(false);
	rgb_spi_handle.BlockingTransmit(reinterpret_cast<uint8_t*>(&rgb_spi_buf), sizeof(rgb_spi_buf));
	rgb_spi_cs.Write(true);
}

uint16_t readKeypadButtonsDownBlocking()
{
	// uses a TCA9555 IO expander on address 0x20 - see https://www.ti.com/lit/ds/symlink/tca9555.pdf
	constexpr uint16_t TCA9555_ADDR = 0x20;

	uint8_t cmd = 0;	// "read input port 0"
	key_i2c_handle.TransmitBlocking(TCA9555_ADDR, &cmd, 1, 1000);

	uint8_t readBuf[2];
	key_i2c_handle.ReceiveBlocking(TCA9555_ADDR, readBuf, sizeof(readBuf), 1000);

	uint16_t down = ~(readBuf[0] | (readBuf[1] << 8));
	return down;
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------

enum class Chord : u8
{
	Major,
	Minor,
	Maj7,
	Min7,
	Maj6,
	Min6,
	Dim,
};

namespace ctrl
{
	int newNoteMidi = 0;
	bool requestNewRoot = false;
	bool playChord = false;

	Chord newChord = Chord::Major;
	bool requestNewChord = false;
}

constexpr int NumOscs = 3;
Oscillator oscBank[NumOscs];
Adsr adsrBank[NumOscs];
ReverbSc reverb;

void audioInit()
{
	for (int i=0; i<NumOscs; ++i)
	{
		oscBank[i].Init(SampleRate);
		oscBank[i].SetWaveform(Oscillator::WAVE_POLYBLEP_TRI);
		oscBank[i].SetFreq(220);

		adsrBank[i].Init(SampleRate, SamplesPerBlock);
		adsrBank[i].SetAttackTime(0.25f + i*0.2f, 0.f);
		adsrBank[i].SetDecayTime(1.2f);
		adsrBank[i].SetSustainLevel(0.7f - i*0.05f);
		adsrBank[i].SetReleaseTime(0.4f);
	}

	reverb.Init(SampleRate);
	reverb.SetFeedback(0.8f);
	reverb.SetLpFreq(1200.f);
}

void audioTick(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	// are we changing chord?
	if (ctrl::requestNewRoot /*|| ctrl::requestNewChord*/)
	{
		u8 newNote0 = ctrl::newNoteMidi;
		u8 newNote1 = newNote0 + 7;
		u8 newNote2 = newNote0 - 12;
		switch (ctrl::newChord)
		{
			case Chord::Major:
				newNote1 = newNote0 + 4;
				newNote2 = newNote0 + 7;
				break;

			case Chord::Minor:
				newNote1 = newNote0 + 3;
				newNote2 = newNote0 + 7;
				break;

			case Chord::Maj7:
				newNote1 = newNote0 + 7;
				newNote2 = newNote0 + 11;
				break;

			case Chord::Min7:
				newNote1 = newNote0 + 7;
				newNote2 = newNote0 + 10;
				break;

			case Chord::Maj6:
				newNote1 = newNote0 + 4;
				newNote2 = newNote0 + 9;
				break;

			case Chord::Min6:
				newNote1 = newNote0 + 4;
				newNote2 = newNote0 + 8;
				break;

			case Chord::Dim:
				newNote1 = newNote0 + 3;
				newNote2 = newNote0 + 6;
				break;
		}

		oscBank[0].SetFreq(mtof(newNote0));
		oscBank[1].SetFreq(mtof(newNote1));
		oscBank[2].SetFreq(mtof(newNote2));

		ctrl::requestNewRoot = false;
		ctrl::requestNewChord = false;
	}

	float env[NumOscs];
	for (int i=0; i<NumOscs; ++i)
	{
		env[i] = adsrBank[i].Process(ctrl::playChord);
	}

	for (size_t i = 0; i < size; i++)
	{
		float o = 0.f;
		for (int i=0; i<NumOscs; ++i)
		{
			o += oscBank[i].Process();
			o *= env[i];
		}
		o *= (1.f / NumOscs);

		float rl, rr;
		reverb.Process(o, o, &rl, &rr);

		out[0][i] = 0.6f * o + 0.4f * rl;
		out[1][i] = 0.6f * o + 0.4f * rr;
	}
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------

int main(void)
{
	hw.Init();

	initKeypad();
	audioInit();

	hw.SetAudioBlockSize(SamplesPerBlock); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAudio(audioTick);

	// set up our basic keyboard colours
	// keys C -> B
	setKeypadPixel(3, 3, 255, 8, 8);
	setKeypadPixel(2, 3, 255, 32, 0);
	setKeypadPixel(3, 2, 255, 61, 0);
	setKeypadPixel(2, 2, 255, 95, 0);
	setKeypadPixel(3, 1, 255, 136, 0);
	setKeypadPixel(2, 1, 255, 186, 0);
	setKeypadPixel(3, 0, 244, 224, 0);
	// sharp key
	setKeypadPixel(2, 0, 0, 20, 130);
	// min / 7 / 6
	setKeypadPixel(0, 3, 0, 163, 254);
	setKeypadPixel(0, 2, 196, 74, 255);
	setKeypadPixel(0, 1, 15, 255, 197);
	//setKeypadPixel(0, 3, 255, 239, 207);

	u16 oldKeysDown = 0;
	u8 currNote = 0;
	Chord currChord = Chord::Major;

	for (;;)
	{
		u16 keysDown = readKeypadButtonsDownBlocking();
		u16 keysChanged = keysDown ^ oldKeysDown;

		if (keysChanged & 0xccc8)
		{
			u8 oldNote = currNote;

			if (keysDown & 0x8000)
				currNote = 48;	// C
			else if (keysDown & 0x4000)
				currNote = 50;	// D
			else if (keysDown & 0x0800)
				currNote = 52;	// E
			else if (keysDown & 0x0400)
				currNote = 53;	// F
			else if (keysDown & 0x0080)
				currNote = 55;	// G
			else if (keysDown & 0x0040)
				currNote = 57;	// A
			else if (keysDown & 0x0008)
				currNote = 59;	// B

			if ((keysDown & 0x0004) && (keysDown & 0xccc8))
				currNote++;

			if (oldNote != currNote)
			{
				ctrl::newNoteMidi = currNote;
				ctrl::requestNewRoot = true;
			}
		}
		if (keysDown & 0xccc8)
		{
			ctrl::playChord = true;
		}
		else
		{
			currNote = 0;
			ctrl::playChord = false;
		}

		Chord oldChord = currChord;
		switch (keysDown & 0x1110)
		{
			case 0: 		currChord = Chord::Major;	break;
			case 0x1000: 	currChord = Chord::Minor;	break;
			case 0x0100: 	currChord = Chord::Maj7;	break;
			case 0x1100: 	currChord = Chord::Min7;	break;
			case 0x0010: 	currChord = Chord::Maj6;	break;
			case 0x1010: 	currChord = Chord::Min6;	break;
			case 0x0110: 	currChord = Chord::Dim;		break;
			default:		currChord = oldChord;		break;
		}
		if (oldChord != currChord)
		{
			ctrl::newChord = currChord;
			ctrl::requestNewChord = true;
		}

		int mask = 1;
		for (u32 y=0; y<4; ++y)
		{
			for (u32 x=0; x<4; ++x, mask <<= 1)
			{
				bool down = (keysDown & mask) != 0;
				setKeypadPixelIntensity(x, y, down ? 11 : RgbSpiBuf::BaseIntensity);
			}
		}

		oldKeysDown = keysDown;

		updateKeypadBlocking();
	}
}
