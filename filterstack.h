#pragma once

#include <cmath>
#include "mln_core.h"

namespace mln
{

enum class FilterType { LoPass, HiPass };

template<unsigned Count, FilterType Type=FilterType::LoPass>
class FilterStack
{
public:
	void Init(float sampleRate)
	{
		m_rSampleRate = 1.f / sampleRate;
		m_b1 = 0.f;
		m_a0 = 1.f;

        for (float& z : m_zN)
		    z = 0.f;
	}

	float Process(float in)
	{
        float z0 = in;
        float out = 0.f;
        for (float& z1 : m_zN)
        {
            out = (z0 * m_a0) + (z1 * m_b1);

            z1 = out;
            z0 = z1;
        }

		if constexpr(Type == FilterType::HiPass)
			return in - out;

		return out;
	}

	void SetFreq(float cutoff)
	{
		float samplesPerHz = cutoff * m_rSampleRate;
		m_b1 = expf(-TWOPI_F * samplesPerHz);
		m_a0 = 1.f - m_b1;
	}

private:
	float m_rSampleRate = 1.f / 48000.f;
	float m_a0 = 1.f, m_b1 = 0.f;	// coefficients
	float m_zN[Count];
};

};
