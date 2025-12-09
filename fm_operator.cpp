#include "fm_operator.h"

namespace mln
{

//----------------------------------------------------------------------------------------------------------------

void FmOperator::Init()
{
    m_env.Init(kSampleRate);
    
    m_modIndex = 1.f;
    m_freq = 220.f;
    m_phase = 0.f;
}

//----------------------------------------------------------------------------------------------------------------

void FmOperator::SetIndex(float modIndex)
{
    m_modIndex = modIndex;
}

//----------------------------------------------------------------------------------------------------------------

void FmOperator::SetEnv(float attack, float decay)
{
    m_env.SetTime(daisysp::ADENV_SEG_ATTACK, attack);
    m_env.SetTime(daisysp::ADENV_SEG_DECAY, decay);
}

//----------------------------------------------------------------------------------------------------------------

float FmOperator::Process(float fm)
{
    const float freq = m_freq + fm * m_freq;
    const float phaseIncPerSample = freq * kRecipSampRate;    

    m_phase += phaseIncPerSample;
    if (m_phase < 0.f) [[unlikely]]
        m_phase += 1.f;
    else if (m_phase >= 1.f) [[unlikely]]
        m_phase -= 1.f;
        
    m_env.Process();
    
    return sinf(kTwoPi * m_phase) * m_env.GetValue();
}

//----------------------------------------------------------------------------------------------------------------

void FmOperator::NoteOn(float baseFreq)
{
    m_freq = baseFreq * m_modIndex;
    m_phase = 0.f;

    m_env.Trigger();
}

//----------------------------------------------------------------------------------------------------------------

} // ns

