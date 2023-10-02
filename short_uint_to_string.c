/*
 * Conversion of the number to string to display it
 *
 * Full "true" number_to_sting conversions:
 * https://github.com/client9/stringencoders
 */

// Convert 16-bit number into hex formatted string
char* usitohex(char* str, const unsigned short int value) {
    static const char hexchars[] = "0123456789ABCDEF";
    str[0] = hexchars[(value >> 12) & 0x000F];
    str[1] = hexchars[(value >> 8) & 0x000F];
    str[2] = hexchars[(value >> 4) & 0x000F];
    str[3] = hexchars[(value) & 0x000F];
    str[4] = '\0';
    return str;
}

static void strreverse(char* begin, char* end) {
    char aux;
    while (end > begin)
        aux = *end, *end-- = *begin, *begin++ = aux;
}

// Convert unsigned short int to string.
// Returns: number of the decimal sings in the number.
// Set shift = 48 for casual string representation, ASCII code.
// Set shift = 0 for raw formatting (intended for display devices).
char uitoa10(char* str, unsigned short int value, char shift) {
    char* wstr = str;
    // Conversion. Number is reversed.
    do
        *wstr++ = (char)(shift + (value % 10));
    while (value /= 10);
    *wstr = '\0';
    // Reverse string
    strreverse(str, wstr - 1);
    return (char)(wstr - str);
}
