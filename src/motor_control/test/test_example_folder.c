#include "example_folder_include.h"
#include "unity.h"

void setUp (void) {} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */

void run_test_add_two(void);
void run_test_add_two(void) {

    int a = 1;
    TEST_ASSERT( 4 == (int)add_two(2) );
    TEST_ASSERT( a == 1 );
}

int main(void) {
    printf("\n\n");
    UNITY_BEGIN();
    RUN_TEST(run_test_add_two);
    UNITY_END();
    return 0;
}