/* This function converts an integer value to a char pointer
 * Made possible with help from : 
 *  https://stackoverflow.com/questions/8257714/how-to-convert-an-int-to-string-in-c*/
 * https://github.com/tmdarwen/STM32/blob/master/STM32F411/OnboardGyroAccel/ 
 */

#define BASE_10  10

char* IntegerToString(int value, char *result, int base);
