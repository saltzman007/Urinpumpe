#ifndef URINPUMPE_H_INCLUDED
#define URINPUMPE_H_INCLUDED

void UrinSoftwarePWM();
void WriteSevenSegment(int i);
void Pulse(int pin);
bool BinIchDran(unsigned long waitTime, unsigned long* p_oldTime);
#endif