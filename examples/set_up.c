
#include <unistd.h>

void play_sound() {
    execlp("aplay", "aplay", "La_Marseillaise.wav", (char *)NULL);
}