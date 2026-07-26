// EXCERPT — source: libClearCore/inc/TrigLUT.h
// EVIDENCE: E1 | symbol: SinQx | lines: 37-59

/**
    \brief Trigonometric lookup table size
    Table contains values for angles 0 to 2π (0 to 32768 in Q15)
**/
#define TRIG_LUT_SIZE 1024

/**
    \brief Get sine value from lookup table
    
    \param[in] angleQx Angle in Q15 format (0-32768 represents 0-2π)
    
    \return Sine value in Q15 format (-32768 to +32768 represents -1.0 to +1.0)
**/
int32_t SinQx(int32_t angleQx);

/**
    \brief Get cosine value from lookup table
    
    \param[in] angleQx Angle in Q15 format (0-32768 represents 0-2π)
    
    \return Cosine value in Q15 format (-32768 to +32768 represents -1.0 to +1.0)
**/
int32_t CosQx(int32_t angleQx);
