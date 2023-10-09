/*
 * StandarLibrary.cpp
 *
 *  Created on: 19-May-2023
 *      Author: aditayagarg
 */

#include <StandarLibrary.h>

int strcasecmp(const char* str1, const char* str2) {
  while((*str1 == *str2 ||  ((*str1) + 32 == *str2) || (*str1 == (*str2) + 32))
      && *str1 != '\0' && *str2 != '\0'
      )
  {str1++;str2++;}
  return *str1 - *str2;
}
