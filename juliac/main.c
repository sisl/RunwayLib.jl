/* #include <stdio.h> */
/* #include "libposeest.h" */

/* int main() { */
/*     printf("Calling test_estimators()...\n"); */
/*     int result; */
/*     for (int i = 0; i < 1; i++) */
/*         result = test_estimators(); */
/*     printf("Result: %d\n", result); */
/*     return 0; */
/* } */

#include <stdio.h>
#include <time.h>
#include "libposeest.h"

int main() {
    clock_t start, end;

    // First call (includes initialization)
    start = clock();
    test_estimators();
    end = clock();
    printf("First call: %.6f seconds\n", ((double)(end - start)) / CLOCKS_PER_SEC);

    // Subsequent calls
    for(int i = 0; i < 5; i++) {
        start = clock();
        test_estimators();
        end = clock();
        printf("Call %d: %.6f seconds\n", i+2, ((double)(end - start)) / CLOCKS_PER_SEC);
    }

    return 0;
}
