#include <inttypes.h>

/// Savitzky Golay Filter
// SGA Coefficients https://en.wikipedia.org/wiki/Savitzky–Golay_filter



class SGAFilter {
public:
    SGAFilter(uint8_t polynomialDegree, uint8_t windowSize);
    int16_t filter(int16_t current_value);
private:
    SAGFilter() {};
    uint8_t _polynomialDegree;
    uint8_t _windowSize;
    uint8_t _index;
    uint8_t _offset;
    uint8_t _middle;
    int16_t * _history;
} ;


