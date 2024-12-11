#pragma once


#include "types.h"
#include <string.h>

void debug_parse_message(uint8_t* _data_buffer, const TypeInfo_t *_type_info);
void debug_parse_member(MemberInfo_t _member_info, uint8_t * _member_data);
void debug_parse_basic_value(MemberInfo_t _member_info, const uint8_t * _member_data);

char* member_type_to_string(MemberInfo_t _member_info);

void dynamic_array_to_yaml(MemberInfo_t member_info,
                           const uint8_t * member_data);

void member_to_array_item(MemberInfo_t member_info,
                          const uint8_t * member_data);

  
  // Convert a dynamically-sized sequence to YAML - implementation function
/*template<typename T>
void
dynamic_array_to_yaml_impl(MemberInfo_t _member_info, T * _array)
{
  for (size_t ii = 0; ii < _array->size; ++ii) {
    member_to_array_item(_member_info, reinterpret_cast<const uint8_t *>(&_array->data[ii]));
  }
}*/


size_t size_of_member_type(uint8_t type_id);
