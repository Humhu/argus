# A single parameter that can take on one of each type, much like a C-union

uint8 PARAM_INVALID=0
uint8 PARAM_INT=1
uint8 PARAM_FLOAT=2
uint8 PARAM_STRING=3
uint8 PARAM_BOOL=4

uint8 type
int64 int_value
float64 float_value
string string_value
bool bool_value