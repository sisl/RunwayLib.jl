#include <stdio.h>
#include "libposeest.h"

int main() {
    printf("Calling test_estimators()...\n");
    int result;
    for (int i = 0; i < 1; i++)
        result = test_estimators();
    printf("Result: %d\n", result);
    return 0;
}
