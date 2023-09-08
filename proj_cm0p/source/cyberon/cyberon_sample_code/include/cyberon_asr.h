/*
PLEASE READ THE CYBERON END USER LICENSE AGREEMENT ("LICENSE(Cyberon)") CAREFULLY BEFORE DOWNLOADING, INSTALLING, COPYING, OR USING THIS SOFTWARE AND ACCOMPANYING DOCUMENTATION.
BY DOWNLOADING, INSTALLING, COPYING OR USING THE SOFTWARE, YOU ARE AGREEING TO BE BOUND BY THE AGREEMENT.
IF YOU DO NOT AGREE TO ALL OF THE TERMS OF THE AGREEMENT, PROMPTLY RETURN AND DO NOT USE THE SOFTWARE.
*/

#ifndef CYBERON_ASR_H_
#define CYBERON_ASR_H_

#include "DSpotterSDKApi.h"

#if defined(__cplusplus)
extern "C" {
#endif

typedef void (*cyberon_asr_callback)(const char *lpchFunction, char *lpchMessage, char *lpchParameter);

/** Initialize automatic speech recognition
 *
 * @param[in] lpfnCallback  The callback of automatic speech recognition
 * @return The status of initialization
 */
BOOL cyberon_asr_init(cyberon_asr_callback lpfnCallback);

/** Do automatic speech recognition
 *
 * @param[in] lpsSample  The pointer of voice data buffer
 * @param[in] nNumSample  The number of voice data (a unit is a short, we prefer to add 480 samples per call)
 */
void cyberon_asr_process(short *lpsSample, int nNumSample);

/** Release automatic speech recognition
 *
 */
void cyberon_asr_release();

#if defined(__cplusplus)
}
#endif

#endif /* CYBERON_ASR_H_ */
