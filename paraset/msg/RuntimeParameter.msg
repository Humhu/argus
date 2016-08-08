# A single parameter that can take on one of each type, much like a C-union

uint8 PARAM_INVALID=0
uint8 PARAM_NUMERIC=1
uint8 PARAM_STRING=2
uint8 PARAM_BOOLEAN=3

uint8 type
float64 numeric_value
string string_value
bool boolean_value