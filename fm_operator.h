#pragma once

#include "mln_core.h"


namespace mln
{

class FmOperator
{
public:
	void Init();

    void SetIndex(float modIndex);
    void SetEnv(float attack, float decay);

	float Process(float fm);

	void NoteOn(float baseFreq);

private:
	daisysp::AdEnv m_env;
    
    float m_modIndex = 1.f;
    float m_freq = 220.f;
    float m_phase = 0.f;
};

}

