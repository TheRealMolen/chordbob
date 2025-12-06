#include "daisy_seed.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisysp;

using u32 = uint32_t;

DaisySeed hw;

SpiHandle rgb_spi_handle;
GPIO rgb_spi_cs;
struct RgbSpiBuf
{
	static constexpr uint32_t Width = 4;
	static constexpr uint32_t Height = 4;
	static constexpr uint32_t NumLeds = Width * Height;
	static constexpr uint32_t RgbiMask = 0x000000e0;	// all ibgr values must be ORd with this

	uint32_t startFrame = 0;

	uint32_t rgbi[NumLeds];

	uint32_t endFrame = ~0u;
};
RgbSpiBuf rgb_spi_buf;


void updateKeypadBlocking();

void initKeypad()
{
	// init our buffer
	for (uint32_t i=0; i < RgbSpiBuf::NumLeds; ++i)
		rgb_spi_buf.rgbi[i] = (42 << 24) | (38 << 16) | (30 << 8) | RgbSpiBuf::RgbiMask | (11);

	// init our SPI setup so we can set RGB values
	SpiHandle::Config cfg;

	cfg.mode = SpiHandle::Config::Mode::MASTER;
	cfg.periph = SpiHandle::Config::Peripheral::SPI_1;

	cfg.pin_config.sclk = seed::D8;
	cfg.pin_config.miso = Pin();	// unused
	cfg.pin_config.mosi = seed::D10;
	cfg.pin_config.nss = Pin();//seed::D7;	// fingers crossed we can just use the HW for this - might need a pullup to 5V

	cfg.direction = SpiHandle::Config::Direction::TWO_LINES_TX_ONLY;	// it's a write-only device

	//cfg.nss = SpiHandle::Config::NSS::HARD_OUTPUT;

	rgb_spi_handle.Init(cfg);

	rgb_spi_cs.Init(seed::D7, GPIO::Mode::OUTPUT);
	rgb_spi_cs.Write(true);

	updateKeypadBlocking();
}

void setKeypadBrightness(uint32_t brightness)
{
	if (brightness >= 32)
		return;

	for (uint32_t i=0; i < RgbSpiBuf::NumLeds; ++i)
	{
		const uint32_t oldRgbi = rgb_spi_buf.rgbi[i];
		const uint32_t oldRgbx = oldRgbi & 0xffffff00;
		rgb_spi_buf.rgbi[i] = RgbSpiBuf::RgbiMask | (brightness) | oldRgbx;
	}
}

void setKeypadPixel(uint32_t x, uint32_t y, uint8_t r, uint8_t g, uint8_t b)
{
	if (x >= RgbSpiBuf::Width || y >= RgbSpiBuf::Height)
		return;
	
	uint32_t i = x + (y * RgbSpiBuf::Width);

	const uint32_t oldRgbi = rgb_spi_buf.rgbi[i];
	const uint32_t oldI = oldRgbi & 0xff;
	const uint32_t rgbx = (uint32_t(r) << 24) | (uint32_t(g) << 16) | (uint32_t(b) << 8);
	rgb_spi_buf.rgbi[i] = oldI | rgbx;
}


void updateKeypadBlocking()
{
	rgb_spi_cs.Write(false);
	rgb_spi_handle.BlockingTransmit(reinterpret_cast<uint8_t*>(&rgb_spi_buf), sizeof(rgb_spi_buf));
	rgb_spi_cs.Write(true);
}

//--------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------

void audioTick(AudioHandle::InputBuffer in, AudioHandle::OutputBuffer out, size_t size)
{
	for (size_t i = 0; i < size; i++)
	{
		out[0][i] = in[0][i];
		out[1][i] = in[1][i];
	}
}

int main(void)
{
	hw.Configure();
	hw.Init();

	initKeypad();

	hw.SetAudioBlockSize(4); // number of samples handled per callback
	hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
	hw.StartAudio(audioTick);

	int col = 0;
	for (;;)
	{
		hw.SetLed(true);
		System::Delay(200);
		hw.SetLed(false);
		System::Delay(200);

		// hw.SetLed(true);
		// System::Delay(600);
		// hw.SetLed(false);
		// System::Delay(200);

		for (u32 y=0; y<4; ++y)
		{
			for (u32 x=0; x<4; ++x)
			{
				bool shinyG = ((x+y) & 3) == (col & 3);
				bool shinyR = ((x-y) & 3) == (col & 3);
				setKeypadPixel(x, y, shinyR?0xff:0x40, shinyG?0xff:0x40, 0x38);
			}
		}
		++col;

		updateKeypadBlocking();
	}
}
