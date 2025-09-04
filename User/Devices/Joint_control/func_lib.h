#ifndef _FUNC_LIB_H_
#define _FUNC_LIB_H_

//homemade math functions
#define VAL_LIMIT(val, min, max) \
do {\
if((val) <= (min))\
{\
  (val) = (min);\
}\
else if((val) >= (max))\
{\
  (val) = (max);\
}\
} while(0)\

#define cushionk(a, b, k) ((a) + (k) * ((b) - (a)))

#endif
