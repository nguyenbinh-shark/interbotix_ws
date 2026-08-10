/* Auto-generated demo for FIS "fuzzy_type11". Reads 2 input(s) from argv, prints 1 output(s). */
#include "fuzzy_type11.h"
#include <stdio.h>
#include <stdlib.h>

int main(int argc, char ** argv) {
    if (argc != 2 + 1) {
        fprintf(stderr, "usage: %s e ed\n", argv[0]);
        return 1;
    }
    float in[2];
    for (int i = 0; i < 2; i++) in[i] = (float) atof(argv[i + 1]);
    float out[1];
    fuzzy_type11_eval_core(in, out);
    for (int i = 0; i < 1; i++)
        printf("%g%c", out[i], (i + 1 < 1) ? ' ' : '\n');
    return 0;
}
