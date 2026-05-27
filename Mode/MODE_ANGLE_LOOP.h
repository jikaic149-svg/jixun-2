#ifndef MODE_ANGLE_LOOP_H
#define MODE_ANGLE_LOOP_H

float MODE_ANGLE_LOOP_PitchUpdate(float target_enc_deg);
float MODE_ANGLE_LOOP_YawUpdate(float target_enc_deg);
void MODE_ANGLE_LOOP_PitchReset(void);
void MODE_ANGLE_LOOP_YawReset(void);
void MODE_ANGLE_LOOP_ResetAll(void);

#endif /* MODE_ANGLE_LOOP_H */
